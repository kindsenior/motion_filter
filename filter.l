(load "euslib/jsk/kalmanlib.l")
;; 各種フィルター

(defun coord+ (coord1 coord2)
  (make-coords
   :pos (v+ (send coord1 :worldpos) (send coord2 :worldpos))
   :rot (m+ (send coord1 :worldrot) (send coord2 :worldrot))))
(defun scale-coord (k coord)
  (make-coords
   :pos (scale k (send coord :worldpos))
   :rot (scale-matrix k (send coord :worldrot))))

(defun joint-pos-list+ (joint-pos-list1 joint-pos-list2)
  (mapcar #'v+ joint-pos-list1 joint-pos-list2))
(defun scale-joint-pos-list (k joint-pos-list)
  (mapcar #'(lambda (joint-pos) (scale k joint-pos)) joint-pos-list))

(setq *float-color-num* 1  *slide-color-num* 4  *draw-color-num* 2  *fan-color-num* 5 *touch-color-num* 3)

;; ;; 回帰平面の法線ベクトルを求める 平面とyベクトルが平行になる姿勢に未対応
;; (defun calc-regression-plane (pos-list)
;;   (let (diff-pos-mat mean-vec Sxx Syy Szz Sxy Syz Szx S ret)
;;     (setq diff-pos-mat (make-matrix (length pos-list) 3))
;;     (setq mean-vec (vector-mean pos-list));; (print mean-vec)
;;     (dotimes (i (length pos-list)) (setf (matrix-row diff-pos-mat i) (v- (elt pos-list i) mean-vec)) );; (print diff-pos-mat)
;;     ;; 共分散
;;     (setq Sxx (/ (v. (matrix-column diff-pos-mat 0) (matrix-column diff-pos-mat 0)) (length pos-list))
;;           Syy (/ (v. (matrix-column diff-pos-mat 1) (matrix-column diff-pos-mat 1)) (length pos-list))
;;           Szz (/ (v. (matrix-column diff-pos-mat 2) (matrix-column diff-pos-mat 2)) (length pos-list))
;;           Sxy (/ (v. (matrix-column diff-pos-mat 0) (matrix-column diff-pos-mat 1)) (length pos-list))
;;           Syz (/ (v. (matrix-column diff-pos-mat 1) (matrix-column diff-pos-mat 2)) (length pos-list))
;;           Szx (/ (v. (matrix-column diff-pos-mat 2) (matrix-column diff-pos-mat 0)) (length pos-list)) )

;;     (setq ret (matrix-column (m* (inverse-matrix (matrix (float-vector Syy Syz) (float-vector Syz Szz))) (matrix (float-vector Sxy) (float-vector Szx)) ) 0) );; x
;;     ;; (setq ret (matrix-column (m* (inverse-matrix (matrix (float-vector Sxx Szx) (float-vector Szx Szz))) (matrix (float-vector Sxy) (float-vector Syz)) ) 0) );; y
;;     ;; (setq ret (matrix-column (m* (inverse-matrix (matrix (float-vector Sxx Sxy) (float-vector Sxy Syy))) (matrix (float-vector Szx) (float-vector Syz)) ) 0) );; z

;;     ;; (setq a (elt ret 0) b (elt ret 1) )
;;     ;; (setq c (- (elt mean-vec 2) (* a (elt mean-vec 0)) (* b (elt mean-vec 1))))

;;     (normalize-vector (float-vector -1 (elt ret 0) (elt ret 1) ));; x
;;     ;; (normalize-vector (float-vector (elt ret 0) -1 (elt ret 1) )) ;; y
;;     ))

;; ;; 回帰平面に投影
;; (defun project-to-regression-plane (pos-list const-point)
;;   (let (n pos)
;;     (setq n (calc-regression-plane  pos-list));; 回帰平面の法線ベクトル計算
;;     (dotimes (i (length pos-list))
;;       (setq pos (elt pos-list i))
;;       (setf (elt pos-list i)
;;             (float-vector (+ (elt const-point 0) (* (elt n 1) (- (elt pos 1) (elt const-point 1))) (* (elt n 2) (- (elt pos 2) (elt const-point 2))) )
;;                           (elt pos 1)
;;                           (elt pos 2) );; x
;;             ;; (float-vector (elt pos 0)
;;             ;;               (+ (* (elt n 0) (- (elt pos 0) (elt const-point 0))) (elt const-point 1) (* (elt n 2) (- (elt pos 2) (elt const-point 2))) )
;;             ;;               (elt pos 2) );; y
;;             )
;;         )
;;     pos-list
;;     ))

;; 2つのモデルリストの相互の干渉をチェック 干渉していなかったらt
(defun collision-check-func (collision-move-targets collision-check-links collision-distance-thresh)
  (not (some #'null
             (reduce #'(lambda (ret target)
                         (append ret (mapcar #'(lambda (link) (> (car (pqp-collision-distance target link)) collision-distance-thresh) ) collision-check-links) ) )
                     (append (list t) collision-move-targets)  )
             ))
  )

;; foot ik 実行単位
(defun foot-ik (lr depth piv coord val-list &key (state -1) (disp 0)
                   (collision-move-targets nil) (collision-check-links nil) (collision-distance-thresh 10) )
  (cond
   (collision-move-targets
    (foot-ik-collision-check lr depth piv coord val-list collision-move-targets collision-check-links collision-distance-thresh :state state :disp disp) )
   (t (foot-ik-no-collision-check lr depth piv coord val-list :state state :disp disp) ) )
  )
;; foot ik 実行単位 干渉チェックなし
(defun foot-ik-no-collision-check (lr depth piv coord val-list &key (state -1) (disp 0) );; disp 0:全表示 1:接地脚のみ表示 -1:非表示
  (let ( (ik-fail-sign 0) (thre 5) end-link );; ik-fail-sign 失敗段階数 0ならik成功
    ;; (send *robot* lr :angle-vector #f(0.0 0.0 -26.0 50.0 -24.0 0.0))
  
    (when (= (length (elt val-list depth)) 5) (setq thre (elt (elt val-list depth) 4)));; 指定があるときのみ精度代入

    (setq end-link (send *robot* lr :end-coords :parent))
    (when (= (length (send *robot* lr :links)) 7) (setq end-link (send end-link :parent)))

    (when (not (send *robot* :inverse-kinematics coord
                     :move-target (send *robot* lr :end-coords)
                     :link-list (send *robot* :link-list end-link)
                     :translation-axis (elt (elt val-list depth) 0) :rotation-axis (elt (elt val-list depth) 1)
                     :thre thre :warnp nil :debug-view nil ))
      (incf ik-fail-sign)
      (when (or (and  (= disp 1) (or (= state 0) (and (= state 1) (equal lr :lleg)) (and (= state 2) (equal lr :rleg)) ) );; 接地脚のみ表示
                (= disp 0) );; 全表示
          (warning-message (elt (elt val-list depth) 2) (elt (elt val-list depth) 3)) (warn " ~a" piv))
      (if (< (1+ depth) (length val-list)) (setq ik-fail-sign (+ ik-fail-sign (foot-ik-no-collision-check lr (1+ depth) piv coord val-list :state state :disp disp)))))
    ;; 一つ条件の緩いikを実行
    ik-fail-sign
    ))
;; foot ik 実行単位 干渉チェックあり
(defun foot-ik-collision-check (lr depth piv coord val-list collision-move-targets collision-check-links collision-distance-thresh
                                   &key (state -1) (disp 0) );; disp 0:全表示 1:接地脚のみ表示 -1:非表示
  (let ( (ik-fail-sign 0) (thre 5) end-link );; ik-fail-sign 失敗段階数 0ならik成功
    (when (= (length (elt val-list depth)) 5) (setq thre (elt (elt val-list depth) 4)));; 指定があるときのみ精度代入

    (setq end-link (send *robot* lr :end-coords :parent))
    (when (= (length (send *robot* lr :links)) 7) (setq end-link (send end-link :parent)))

    (when (not (send *robot* :inverse-kinematics coord
                     :move-target (send *robot* lr :end-coords)
                     :link-list (send *robot* :link-list end-link)
                     :translation-axis (elt (elt val-list depth) 0) :rotation-axis (elt (elt val-list depth) 1)
                     :thre thre :warnp nil :debug-view nil
                     :avoid-collision-null-gain 100 :avoid-collision-joint-gain 0.1
                     :additional-check #'(lambda () (funcall 'collision-check-func collision-move-targets collision-check-links collision-distance-thresh)) )) 
      (incf ik-fail-sign)
      (when (or (and  (= disp 1) (or (= state 0) (and (= state 1) (equal lr :lleg)) (and (= state 2) (equal lr :rleg)) ) );; 接地脚のみ表示
                (= disp 0) );; 全表示
          (warning-message (elt (elt val-list depth) 2) (elt (elt val-list depth) 3)) (warn " ~a" piv))
      (if (< (1+ depth) (length val-list)) (setq ik-fail-sign (+ ik-fail-sign
                                                                 (foot-ik-collision-check lr (1+ depth) piv coord val-list
                                                                                          collision-move-targets collision-check-links collision-distance-thresh
                                                                                          :state state :disp disp)))));; 一つ条件の緩いikを実行
    ik-fail-sign
    ))

;;足裏フィッティング ik実行
(defun exec-foot-ik ( piv frame-list l-ik-list r-ik-list &key (disp 0) );; disp 0:全表示 1:接地脚のみ表示
  (let 
      (;; (lfoot-pos (send (send *robot* :lleg :end-coords :copy-worldcoords) :worldpos))
       ;; (rfoot-pos (send (send *robot* :rleg :end-coords :copy-worldcoords) :worldpos))
       ;; (lfoot-rpy (car (send (send *robot* :lleg :end-coords :copy-worldcoords) :rpy-angle)))
       ;; (rfoot-rpy (car (send (send *robot* :rleg :end-coords :copy-worldcoords) :rpy-angle)))
       (floor 0)
       (ik-fail-sign-cons (cons nil nil))
       state);;垂直接地開始高さ

    ;; 目標座標代入
    ;; (setq lfoot-pos (send (elt frame-list piv) :pos :lleg))(setq lfoot-rpy (car (send (elt frame-list piv) :rpy-angle)))
    ;; (setq rfoot-pos (send (elt frame-list piv) :pos :rleg))(setq rfoot-rpy (car (send (elt frame-list piv) :rpy-angle)))

    ;; 足目標位置
    (send *lfoot* :move-to (send (elt frame-list piv) :lfoot-coord) :world)(send *lfoot* :locate *lfootprint-offset*)
    (send *rfoot* :move-to (send (elt frame-list piv) :rfoot-coord) :world)(send *rfoot* :locate *rfootprint-offset*)

    ;; state 0:両足接地 1:左足接地 2:右足接地 3:跳躍
    ;; スライドとファンも接地とみなす (jump表示に関して)
    (cond;; 接地足表示・設定
     ((and (= (send (elt frame-list piv) :state :lleg) *ground*) (= (send (elt frame-list piv) :state :rleg) *ground*) ) (format stdout "     both fit~%")(setq state 0));; 両足fit
     ((or= (send (elt frame-list piv) :state :lleg) (list *ground* *slide* *fan*)) (format stdout "     left fit~%") (setq state 1));;左足fit
     ((or= (send (elt frame-list piv) :state :rleg) (list *ground* *slide* *fan*)) (format stdout "     right fit~%")(setq state 2));;右足fit
     (t (warning-message 1 " jump ~a " piv)(setq state 3))
     )

    ;;ik-failするとtranslation-axisをrotation-axisを変更して実行しなおす
    (setf (car ik-fail-sign-cons) (foot-ik :lleg 0 piv (send (elt frame-list piv) :lfoot-coord) l-ik-list :state state :disp disp));; 左足
    (setf (cdr ik-fail-sign-cons) (foot-ik :rleg 0 piv (send (elt frame-list piv) :rfoot-coord) r-ik-list :state state :disp disp));; 右足

    (format stdout "~%")
    ;; (format stdout "     dest lpos ~28a   rpos ~28a ~%" lfoot-pos rfoot-pos)
    (format stdout "   actual lpos ~28a   rpos ~28a ~%"
            (send *robot* :lleg :end-coords :worldpos) (send *robot* :rleg :end-coords :worldpos))

    ;; 干渉確認
    ;; (when (and (not (= (elt lz-list piv) 0))
    ;;            (= (rapid-collision-check *box* (elt (send *robot* :lleg :links) 5) ) 1 ) )
    ;;   (warning-message 1 "lfoot collision cannot resolve")
    ;;   )
    ;; (when (and (not (= (elt rz-list piv) 0))
    ;;            (= (rapid-collision-check *box* (elt (send *robot* :rleg :links) 5)) 1 ) )
    ;;   (warning-message 1 " rfoot collision cannot resolve")
    ;;   )
    ;; (when (= (rapid-collision-check *lfoot* *rfoot*) 1 )
    ;;   (warning-message 1 " feet collision cannot resolve")
    ;; )


    (format stdout "~%")

    ik-fail-sign-cons
    ))

;; coordからposの差分の正規ベクトルを計算
(defun calc-pos-diff (pre-frame-list cur-frame-list limb)
  (let ( (diff-list nil) diff (diff-thresh 2) )
    (dotimes (i (length cur-frame-list))
      (setq diff (v- (send (elt cur-frame-list i) :pos limb) (send (elt pre-frame-list i) :pos limb)))
      (cond
       ( (< (norm diff) diff-thresh) (push #f(0 0 0) diff-list) );; 小さな差分ベクトルは除去 
       (t (push-back (normalize-vector diff) diff-list) ) );; 差分ベクトルは正規化する
      )
    diff-list
    ))

;; 足跡決定 (ik-fail 左右足干渉 回避)
(setq *ik-yaw-fail-step* 1  *ik-pos-fail-step* 2);; ik-failの種類
(defun decide-footprint (frame-list &key (animate nil) (stdout_ t) (floor 0) )
  (let ( (interfere-flg nil) (ik-fail-flg t) touch-fail-flg (init-flg t) (num 0) failnum ik-fail-flg-cons ik-fail-num-list ik-fail-type-list return-list down-flg-list1 down-flg-list-2 disp
         l-ikdiff-list r-ikdiff-list l-interferediff-list r-interferediff-list l-ik-list r-ik-list ik-step-num
         ;; frame-list-dest 
         frame-list-for-legs frame-list-distorted )
    ;; touch-fail-flg （干渉を回避を考慮済み）接地平面への接地そのものができない 水平接地失敗
    ;; ik-fail-flg 水平(r=p=0)接地(z=0)は可能 接地位置・ヨー角は未達成
    (setq stdout stdout_)
    (setq ik-fail-num-list (make-list (length frame-list) :initial-element 0);; ik失敗回数リスト
          robot-z-list-tmp (make-list (length frame-list) :initial-element 600)
          frame-list-for-legs (copy-object frame-list) )

    ;; (setq l-ik-list (list (list t t 2 "  l" 2)(list :xy t 3 " z+rpy")(list t :z 5 "xyz+rp")(list :xy :z 1 " z+rp"));; 精度2mm
    ;;       r-ik-list (list (list t t 4 "  r" 2)(list :xy t 3 " z+rpy")(list t :z 5 "xyz+rp")(list :xy :z 1 " z+rp"))
    ;;       ik-step-num (length l-ik-list)
    ;;       ik-pos-fail-step 1
    ;;       ik-yaw-fail-step 2)
    (setq l-ik-list (list (list t t 2 "  l" 2)(list t :z 5 "xyz+rp")(list :xy t 3 " z+rpy")(list :xy :z 1 " z+rp"));; 精度2mm
          r-ik-list (list (list t t 4 "  r" 2)(list t :z 5 "xyz+rp")(list :xy t 3 " z+rpy")(list :xy :z 1 " z+rp"))
          ik-step-num (length l-ik-list) ;; ikのend-coodsの許容条件の段階数
          )
          ;; ik-pos-fail-step 2
          ;; ik-yaw-fail-step 1)

    ;; (while (and (or ik-fail-flg interfere-flg touch-fail-flg) (or init-flg (> failnum 1) touch-fail-flg) )
    (while (or ik-fail-flg interfere-flg touch-fail-flg)
      (warning-message 3  "loop num ~a~%" num)

      (setq ik-fail-flg nil  failnum 0 touch-fail-flg nil ik-fail-type-list nil)
      (setq down-flg-list1 (make-list (length frame-list) :initial-element nil))
      (setq down-flg-list2 (make-list (length frame-list) :initial-element nil))

      (setq frame-list-dest (copy-object frame-list));; 目標座標保存
      (setq frame-list-distorted (copy-object frame-list));; ik失敗後の座標

      (dotimes (j (length frame-list));; ikがfailしたかどうかをチェック 動作シーケンスを走査
        (format stdout "ik loop num ~3a  time ~A   pseq frame ~A~%" num (* *ft* j) j)

        (send *robot* :angle-vector (send (elt frame-list j) :angle-vector));; 上半身セット
        (send *robot* :move-to (send (elt frame-list j) :waist-coord) :world);; 腰移動
        (cond
         ( (or init-flg touch-fail-flg) (setq disp -1) );; 非表示
         ( t (setq disp 1) ) );; 接地脚のみ表示
        (setq ik-fail-flg-cons (exec-foot-ik j frame-list;; ik実行
                                             l-ik-list r-ik-list
                                             ;; (list (list t t 2 "  l" 2)(list :xy :z 1 " z+rp"));; 精度2mm
                                             ;; (list (list t t 4 "  r" 2)(list :xy :z 1 " z+rp"))
                                             :disp disp) )
        (push-back ik-fail-flg-cons ik-fail-type-list);; 左右足のikの結果のconsのリスト

        (cond
         ;; 水平接地失敗時
         ( (or (and (= (car ik-fail-flg-cons) ik-step-num) (= (send (elt frame-list j) :state :lleg) *ground*)) (and (= (cdr ik-fail-flg-cons) ik-step-num) (= (send (elt frame-list j) :state :rleg) *ground*))
               (and (= (car ik-fail-flg-cons) ik-step-num) (= (send (elt frame-list j) :state :lleg) *slide*))  (and (= (cdr ik-fail-flg-cons) ik-step-num) (= (send (elt frame-list j) :state :rleg) *slide*))
               (and (= (car ik-fail-flg-cons) ik-step-num) (= (send (elt frame-list j) :state :lleg) *fan*))    (and (= (cdr ik-fail-flg-cons) ik-step-num) (= (send (elt frame-list j) :state :rleg) *fan*)) )
           (setq touch-fail-flg t)
           (setf (elt down-flg-list1 j) t);; 腰下げフラグ
           )
         ;; 接地かつik-fail時だけ
         ( (or (and (>= (car ik-fail-flg-cons) 1) (= (send (elt frame-list j) :state :lleg) *ground*)) (and (>= (cdr ik-fail-flg-cons) 1) (= (send (elt frame-list j) :state :rleg) *ground*))
               (and (>= (car ik-fail-flg-cons) 1) (= (send (elt frame-list j) :state :lleg) *slide*))  (and (>= (cdr ik-fail-flg-cons) 1) (= (send (elt frame-list j) :state :rleg) *slide*))
               (and (>= (car ik-fail-flg-cons) 1) (= (send (elt frame-list j) :state :lleg) *fan*))    (and (>= (cdr ik-fail-flg-cons) 1) (= (send (elt frame-list j) :state :rleg) *fan*)) )
           (setq ik-fail-flg t) (incf failnum)
           (if (some #'(lambda (val) (= val *ik-pos-fail-step*)) (elt ik-fail-type-list j)) (setf (elt down-flg-list2 j) t))
           (setf (elt ik-fail-num-list j) (1+ (elt ik-fail-num-list j)))
           )
         (t
          (setf (elt ik-fail-num-list j) 0)
          );; ik成功
         )

        (if (select-stream (list *standard-input*) 1.0e-8) (throw :filter nil))

        ;; ik-fail時、接地足座標
        (when (and (= (car ik-fail-flg-cons) *ik-yaw-fail-step*)
                   (or (= (send (elt frame-list j) :state :lleg) *ground*) (= (send (elt frame-list j) :state :lleg) *slide*) (= (send (elt frame-list j) :state :lleg) *fan*) ) )
          (send (elt frame-list-distorted j) :set-coord :lleg (send *robot* :lleg :end-coords :copy-worldcoords))
          )
        (when (and (= (cdr ik-fail-flg-cons) *ik-yaw-fail-step*)
                   (or (= (send (elt frame-list j) :state :rleg) *ground*) (= (send (elt frame-list j) :state :rleg) *slide*) (= (send (elt frame-list j) :state :rleg) *fan*) ) )
          (send (elt frame-list-distorted j) :set-coord :rleg (send *robot* :rleg :end-coords :copy-worldcoords))
          )
        ;; (when (or (= (send (elt frame-list j) :state :lleg) *ground*) (= (send (elt frame-list j) :state :lleg) *slide*) (= (send (elt frame-list j) :state :lleg) *fan*) )
        ;;   (send (elt frame-list-for-legs j) :set-coord :lleg (send *robot* :lleg :end-coords :copy-worldcoords))
        ;;   )
        ;; (when (or (= (send (elt frame-list j) :state :rleg) *ground*) (= (send (elt frame-list j) :state :rleg) *slide*) (= (send (elt frame-list j) :state :rleg) *fan*) )
        ;;   (send (elt frame-list-for-legs j) :set-coord :rleg (send *robot* :rleg :end-coords :copy-worldcoords))
        ;;   )
        ;; av代入
        (send (elt frame-list j) :angle-vector (send *robot* :angle-vector))

        (when (and animate (> num 10)) (send *pickview* :draw-objects)(unix::usleep (* 1000 400))(x::window-main-one));; 表示
        (if (select-stream (list *standard-input*) 1.0e-8) (throw :filter nil))
        ;; (if (and (> num 10) (= j 35)) (throw :filter nil))
        );; end dotimes   ik-failを探す走査
      (warning-message 1  "~%~a fails~%" failnum)

      ;; ikの失敗の仕方に依って各種修正
      (cond
       ;; 水平接地失敗時
       ( touch-fail-flg
         (setq frame-list (down-waist frame-list down-flg-list1 -10));; キーポーズモードでは要改良
         (setq ik-fail-num-list (make-list (length frame-list) :initial-element 0));; ik-fail-num-listをリセット
         )
       ;; ik失敗時
       ( (not touch-fail-flg)
         ;; (setq frame-list (copy-frame-slot frame-list-for-legs frame-list '(:coord) '(:lleg :rleg)));; 足座標のみコピー
         ;; (setq frame-list (copy-frame-slot frame-list-distorted frame-list '(:coord) '(:lleg :rleg)));; 足座標のみコピー

         (setq l-ikdiff-list (calc-pos-diff frame-list-dest frame-list :lleg)  r-ikdiff-list (calc-pos-diff frame-list-dest frame-list :rleg));; ikdiff計算

         (when ik-fail-flg;; ik-fail回避
           ;; ヨー軸回り回転修正
           ;; (setq frame-list (detect-non-yaw-ik-fail-footprint frame-list frame-list-dest ik-fail-type-list *ik-yaw-fail-step* :floor floor))

           (setq frame-list (detect-non-yaw-ik-fail-footprint frame-list-distorted frame-list ik-fail-type-list :floor floor));; キーポーズモードでは要改良

           (setq frame-list (down-waist frame-list down-flg-list2 -10));; キーポーズモードでは要改良

           ;; 位置修正
           ;; (setq frame-list (detect-non-pos-ik-fail-footprint frame-list ik-fail-num-list :lleg :floor floor))
           ;; (setq frame-list (detect-non-pos-ik-fail-footprint frame-list ik-fail-num-list :rleg :floor floor))
           )

         ;; 干渉回避
         (setq return-list (detect-non-interfere-footprint frame-list :floor floor))
         (setq frame-list (car return-list) interfere-flg (cadr return-list))
         (setq l-interferediff-list (calc-pos-diff frame-list-dest frame-list :lleg)  r-interferediff-list (calc-pos-diff frame-list-dest frame-list :rleg));; interferediff計算
         ;; (warn " interfere ~a~%" interfere-flg)

         ;; 同一支持期中の腰の水平移動量が大きい時の腰下げ
         (setq frame-list (avoid-far-footprint frame-list ik-fail-num-list
                                               l-ikdiff-list r-ikdiff-list l-interferediff-list r-interferediff-list))

         ;; 連続ik-fail時の腰下げ
         (setq frame-list (remove-continual-ik-fail frame-list ik-fail-num-list :min-count 7 :max-count 20))

         (incf num)
         )
       )

      ;; ループごとグラフ表示
      (setq robot-z-list-last (copy-seq robot-z-list-tmp))
      (dotimes (i (length frame-list)) (setf (elt robot-z-list-tmp i) (elt (send (elt frame-list i) :pos :waist) 2)))
      (send *gnuplot* :draw robot-z-list-medi2 robot-z-list-last robot-z-list-tmp)

      (setq init-flg nil)
      );; end while

    (setq stdout nil)
    frame-list
    ))

;; 緩やかな腰下げ
;; 腰を上げるのには未対応
(defun down-waist (frame-list down-flg-list z-step)
  (let* (tmp-list (z-gap (* -1.0 z-step)) (term-time (* z-gap *ft* 10)) adjacence idx-diff)
    (setq tmp-list (copy-object frame-list))

    (dotimes (piv (length frame-list))
      (cond
       ;; down-flgが無いとき
       ( (not (elt down-flg-list piv))
         (setq idx-diff 1 adjacence 0);; 1 <= adjacence <= z-gap  1 <= adjacence <= term-time
         (while (and (< adjacence term-time)
                     (<= 0 (- piv idx-diff) ) (<= (+ piv idx-diff) (1- (length frame-list)) )
                     (or (not (elt down-flg-list (- piv idx-diff))) (not (elt down-flg-list (+ piv idx-diff)))) )
           (setq adjacence (min (- (send (elt frame-list (+ piv idx-diff)) :time) (send (elt frame-list piv) :time))
                                (- (send (elt frame-list piv) :time) (send (elt frame-list (- piv idx-diff)) :time))))
           (incf idx-diff)
           )

         (when (or (and (<= 0 (- piv idx-diff) ) (elt down-flg-list (- piv idx-diff)) )
                   (and (<= (+ piv idx-diff) (1- (length frame-list))) (elt down-flg-list (+ piv idx-diff)) ) )
           ;; (setq adjacence (- z-gap adjacence));; 0 <= 隣接度 < z-gap
           (setq adjacence (max (- term-time adjacence) 0));; 0 <= 隣接度 < term-time

           ;; (send (send (elt tmp-list piv) :coord :waist) :translate (float-vector 0 0 (* z-gap 0.5 (- (cos (* (/ pi z-gap) adjacence)) 1)) ) :world));; 腰下げ
           (send (send (elt tmp-list piv) :coord :waist) :translate (float-vector 0 0 (* z-gap 0.5 (- (cos (* (/ pi term-time) adjacence)) 1)) ) :world));; 腰下げ
         )
       ;; down-flgが有るとき
       ( t (send (send (elt tmp-list piv) :coord :waist) :translate (float-vector 0 0 z-step) :world) );; 腰下げ
       )
      ) ;; end dotimes

      tmp-list
    ))

;; 連続ik-fail時は腰を落とす
(defun remove-continual-ik-fail (frame-list ik-fail-num-list &key (min-count 7) (max-count 20))
  (let ( tmp-list down-flg-list )
    (setq tmp-list (copy-object frame-list))
    (warn "continual fail ")
    (dotimes (piv (length frame-list))
      (cond ( (and (< min-count (elt ik-fail-num-list piv)) (< (elt ik-fail-num-list piv) max-count))
              (warn " ~a" piv)
              (push-back t down-flg-list);; 腰下げフラグ
              )
            (t (push-back nil down-flg-list) ) )
      )
    (warn "~%")
    (setq tmp-list (down-waist frame-list down-flg-list -5));; 腰下げ

    tmp-list
    ))

;; ギャップ緩和 高い方でギャップを緩和
(defun moderate-gap (coord-list  &key (thresh 5))
  (let (coord-list-tmp next upfunc downfunc upstep downstep base end-idx range term idx z)
    (setq coord-list-tmp (copy-object coord-list))
    (dotimes (piv (length coord-list))
      (setq next (min (1+ piv) (1- (length coord-list))))
      (when (> (abs (- (elt (send (elt coord-list piv) :worldpos) 2) (elt (send (elt coord-list next) :worldpos) 2))) thresh)
        (cond
         ( (< (elt (send (elt coord-list piv) :worldpos) 2) (elt (send (elt coord-list next) :worldpos) 2) );; 上昇
           (setq upfunc '1+  downfunc '1-  base (eval piv)  upstep '+  downstep '-) )
         ( t ;; 下降
           (setq upfunc '1-  downfunc '1+  base (eval next)  upstep '-  downstep '+) ) )

        (setq end-idx (funcall upfunc (eval base))  term 1);; 緩和区間決定
        (while (and (> end-idx 0) (< end-idx (1- (length coord-list)))
                    (> (abs (- (elt  (send (elt coord-list base) :worldpos) 2) (elt  (send (elt coord-list end-idx) :worldpos) 2))) (* term thresh)) )
          (setq end-idx (funcall upfunc end-idx)) (incf term))

        ;; range,term再設定
        (setq range (abs (- (elt (send (elt coord-list base) :worldpos) 2) (elt (send (elt coord-list end-idx) :worldpos) 2)))
              term (/ range thresh))

        ;; 緩衝化軌道代入
        (setq idx (funcall upfunc (eval base)))
        (while (<= (* (- idx base) (- idx end-idx)) 0)
          (setq z (+ (elt (send (elt coord-list base) :worldpos) 2) (* (/ (abs (- idx base)) (float term)) range)))
          (setf (elt coord-list-tmp idx) (make-coords :pos (float-vector (elt (send (elt coord-list idx) :worldpos) 0) (elt (send (elt coord-list idx) :worldpos) 1) z)
                                                      :rot (send (elt coord-list idx) :worldrot)))
          (setq idx (funcall upfunc idx)) )
        );; end when
      );; end dotimes

    coord-list-tmp
    ))

;; 同一支持期中の腰の水平移動量が大きい時の腰下げ
(defun avoid-far-footprint (frame-list ik-fail-num-list l-ikdiff-list r-ikdiff-list l-interferediff-list r-interferediff-list)
  (let ( (piv 0) diff-list lidx ridx endidx ikdiff-list interferediff-list idx tmp-list down-flg-list )
    (setq tmp-list (copy-object frame-list)
          down-flg-list (make-list (length frame-list) :initial-element nil))
    (warn "down waist ")

    (while (< piv (length frame-list));; リスト走査
      (setq diff-list nil  ikdiff-list nil  interferediff-list nil)

      (cond
       ( (or (= (send (elt frame-list piv) :state :lleg) *ground*) (= (send (elt frame-list piv) :state :rleg) *ground*) )
         (setq lidx (eval piv)  ridx (eval piv) );; 支持脚期間が後まで続く脚を識別
         (while (and (< lidx (length frame-list)) (= (send (elt frame-list lidx) :state :lleg) *ground*)  ) (incf lidx))
         (while (and (< ridx (length frame-list)) (= (send (elt frame-list ridx) :state :rleg) *ground*)  ) (incf ridx))
         (cond ( (< lidx ridx) (setq ikdiff-list r-ikdiff-list  interferediff-list r-interferediff-list  endidx ridx ) )
               (t (setq ikdiff-list l-ikdiff-list  interferediff-list l-interferediff-list  endidx lidx) ) )

         (setq idx (eval piv));; 差分ベクトルリスト作成 正規化済み
         (while (< idx endidx) (push (elt ikdiff-list idx) diff-list) (push (elt interferediff-list idx) diff-list) (incf idx))

         (setq idx (eval piv))
         (when (< (v. (car (farthest-pair diff-list)) (cadr (farthest-pair diff-list))) (cos (deg2rad 135)) );; 最遠の差分ベクトルの組のなす角が90度以上の時
           ;; (warn " piv ~a  end ~a  " piv endidx)
           (while (< idx endidx)
             (when (> (elt ik-fail-num-list idx) 0);; ik-failの時のみ腰下げ
               (warn " ~a" idx)
               (when (or (= idx 40) (= idx 54)) (warn " ~a~%" diff-list))
               (setf (elt down-flg-list idx) t) );; 腰下げフラグ
             (incf idx))
           ;; (warn "~%")
           );; 支持脚期間の間
         (setq piv (1+ endidx))
         )
       (t (incf piv)))

      );;end リスト走査
    (warn "~%")
    (setq tmp-list (down-waist frame-list down-flg-list -10));; 腰下げ

    tmp-list
    ))

;; 非干渉足跡探査
(defun detect-non-interfere-footprint ( frame-list &key (floor 0) )
  (let ( tmp-list (interfere-flg nil) (piv 0) local-interfere-flg step-vec (step-size 2)
                   lidx ridx l r lpos rpos wpos  fx fy nx ny  fpos npos far near f n  (collision-distance-thresh 50) )
    (warn "interfere" )
    (setq tmp-list (copy-object frame-list))
    (while (< piv (length frame-list))
      (setq local-interfere-flg nil)
      ;; (print piv)
      (cond
       ;; 両足接地時
       ((and (= (send (elt frame-list piv) :state :lleg) *ground*) (= (send (elt frame-list piv) :state :rleg) *ground*))

        (send *lfoot* :move-to (send (elt frame-list piv) :lfoot-coord) :world)(send *lfoot* :locate *lfootprint-offset*)
        (send *rfoot* :move-to (send (elt frame-list piv) :rfoot-coord) :world)(send *rfoot* :locate *rfootprint-offset*)

        ;; (while (= (rapid-collision-check *lfoot* *rfoot*) 1);; 干渉回避
        (while (< (car (pqp-collision-distance *lfoot* *rfoot*))  collision-distance-thresh);; 干渉回避
          (setq interfere-flg t  local-interfere-flg t)

          (setq wpos (send (elt frame-list piv) :pos :waist))(setf (elt wpos 2) 0)
          (setq lpos (send (elt frame-list piv) :pos :lleg))(setf (elt lpos 2) 0)
          (setq rpos (send (elt frame-list piv) :pos :rleg))(setf (elt rpos 2) 0)
          (setq l (norm (v- lpos wpos))) (setq r (norm (v- rpos wpos)))

          ;; 左右足の遠近チェック
          (cond ((> l r) (setq f l)(setq n r)(setq fpos lpos)(setq npos rpos)(setq far *lfoot*)(setq near *rfoot*))
                (t (setq f r)(setq n l)(setq fpos rpos)(setq npos lpos)(setq far *rfoot*)(setq near *lfoot*)))

          (setq step-vec (scale step-size (normalize-vector (v- npos fpos))));; 遠足から近足への単位ベクトル
          (send near :translate  step-vec :world)
          (send far :translate (scale -1.0 step-vec) :world)

          );; end while
        (send *lfoot* :locate (scale -1 *lfootprint-offset*))(send *rfoot* :locate (scale -1 *rfootprint-offset*))

        ;; ;; ikのz方向の誤差をなくす これをしたければ floor 引数が必要
        ;; (setq lpos (send *lfoot* :worldpos)  rpos (send *rfoot* :worldpos))
        ;; (send *lfoot* :locate (float-vector (elt lpos 0) (elt lpos 1) floor) :world)
        ;; (send *rfoot* :locate (float-vector (elt rpos 0) (elt rpos 1) floor) :world)

        (setq lidx (eval piv));; 左足跡更新
        (while (and (> lidx 0) (= (send (elt frame-list (1- lidx)) :state :lleg) *ground*)) (decf lidx));; lidx移動
        (while (and (< lidx (length frame-list)) (= (send (elt frame-list lidx) :state :lleg) *ground*)) (send (send (elt tmp-list lidx) :lfoot-coord) :locate (send *lfoot* :worldpos) :world)(incf lidx))
        (setq ridx (eval piv));; 右足跡更新
        (while (and (> ridx 0) (= (send (elt frame-list (1- ridx)) :state :rleg) *ground*)) (decf ridx));; ridx移動
        (while (and (< ridx (length frame-list)) (= (send (elt frame-list ridx) :state :rleg) *ground*)) (send (send (elt tmp-list ridx) :rfoot-coord) :locate (send *rfoot* :worldpos) :world)(incf ridx))

        (if local-interfere-flg (warn "   from ~a to ~a" piv (max lidx ridx)))
        (setq piv (min lidx ridx));; (warn " ~a ~a~%" lidx ridx)
        );; 両足接地時 終了

       ;; スライド,ファン??

       ;; 非接地期
       (t (incf piv))
       )
      )

    (if (not interfere-flg) (warn " nan"))
    (warn "~%")

    (list tmp-list interfere-flg)
    ))

;; 1 対 多数 での値比較
(defun or= (val target-list)
  (some #'(lambda (target) (= val target)) target-list))

;; ik-fail足跡(yaw軸回り回転)除去
;; yawの最大開き角度が腰の姿勢に依って異なることには未対応
(defun detect-non-yaw-ik-fail-footprint ( frame-list frame-list-dest ik-fail-type-list &key (floor 0) )
  (let ( tmp-list (piv 0) lr-yaw-diff on-yaw-diff lidx ridx  lyaw ryaw wyaw )
    (warn "yaw fail" )
    ;; (setq tmp-list (copy-object frame-list-dest))
    (setq tmp-list (copy-seq frame-list-dest))
    ;; (setq tmp-list (copy-object frame-list))
    (while (< piv (length frame-list))
      (setq local-interfere-flg nil)
      ;; (print piv)
      (cond
       ;; 両足接地時 ik-fail-typeは接地しているかどうかは判別していない
       ( (and (or= (send (elt frame-list piv) :state :lleg) (list *ground* *slide* *fan*)) (or= (send (elt frame-list piv) :state :rleg) (list *ground* *slide* *fan*))
              (some #'(lambda (type) (= type *ik-yaw-fail-step*)) (elt ik-fail-type-list piv)))
         (setq lr-yaw-diff (car (car (send (send (send (elt frame-list-dest piv) :coord :rleg) :transformation (send (elt frame-list-dest piv) :coord :lleg) :local) :rpy-angle))));; 左右足先座標でのyaw-diff
         (warn "~%before:~a~%" (rad2deg lr-yaw-diff))
         (cond
          ;; 股のyawの開きが大きい時
          ( (> lr-yaw-diff *crotch-yaw-max-angle*)
            (setq lyaw (car (car (send (send (elt frame-list-dest piv) :coord :lleg) :rpy-angle)));; 左足yaw
                  ryaw (car (car (send (send (elt frame-list-dest piv) :coord :rleg) :rpy-angle))));; 右足yaw

            ;; 左足跡更新
            (setq lidx (eval piv))
            (cond
             ( (= (send (elt frame-list piv) :state :lleg) *ground*);; 面接地
               (while (and (> lidx 0) (= (send (elt frame-list (1- lidx)) :state :lleg) *ground*)) (decf lidx));; lidx移動
               (while (and (< lidx (length frame-list)) (= (send (elt frame-list lidx) :state :lleg) *ground*)) 
                 (send (send (elt tmp-list lidx) :coord :lleg) :rpy (- lyaw (/ (- lr-yaw-diff *crotch-yaw-max-angle*) 2.0)) 0 0)(incf lidx)) )
             ( t (send (send (elt tmp-list piv) :coord :lleg) :rpy (- lyaw (/ (- lr-yaw-diff *crotch-yaw-max-angle*) 2.0)) 0 0)(incf lidx) ) ;; スライド ファン
             )

            ;; 右足跡更新
            (setq ridx (eval piv))
            (cond
             ( (= (send (elt frame-list piv) :state :rleg) *ground*);; 面接地
               (while (and (> ridx 0) (= (send (elt frame-list (1- ridx)) :state :rleg) *ground*)) (decf ridx));; ridx移動
               (while (and (< ridx (length frame-list)) (= (send (elt frame-list ridx) :state :rleg) *ground*))
                 (send (send (elt tmp-list ridx) :coord :rleg) :rpy (+ ryaw (/ (- lr-yaw-diff *crotch-yaw-max-angle*) 2.0)) 0 0)(incf ridx)) )
             ( t (send (send (elt tmp-list piv) :coord :rleg) :rpy (+ ryaw (/ (- lr-yaw-diff *crotch-yaw-max-angle*) 2.0)) 0 0)(incf ridx) );; スライド ファン
             )

            (warn "   lleg:~a〜~a rleg:~a〜~a" piv (1- lidx) piv (1- ridx))
            ;; (warn " ~a-~a ~a+~a "lyaw (/ (- lr-yaw-diff *crotch-yaw-max-angle*) 2.0) ryaw (/ (- lr-yaw-diff *crotch-yaw-max-angle*) 2.0))
            (setq piv (min lidx ridx));; (warn " ~a ~a~%" lidx ridx)
            )
          ;; 股のyawの開きが小さい時
          ( t
            (cond
             ( (= (car (elt ik-fail-type-list piv)) *ik-yaw-fail-step*);; 左足
               (setq on-yaw-diff (car (car (send (send (send (elt frame-list piv) :coord :lleg) :transformation (send (elt frame-list-dest piv) :coord :lleg) :local) :rpy-angle))));; 新旧足先座標でのyaw-diff
               (setq wyaw (car (car (send (send (elt frame-list piv) :coord :waist) :rpy-angle))));; 腰yaw
               (send (send (elt tmp-list piv) :coord :waist) :rpy (+ wyaw on-yaw-diff) 0 0);; 足ずれた方と反対方向に腰を捻る
               ;; (send (elt tmp-list piv) :coord :lleg (send (elt frame-list-dest piv) :coord :lleg));; ずれた足を目標座標に戻す
               (warn "  ~a(~a)" piv :leg)
               (incf piv)
               )
             ( (= (cdr (elt ik-fail-type-list piv)) *ik-yaw-fail-step*);; 右足
               (setq on-yaw-diff (car (car (send (send (send (elt frame-list piv) :coord :rleg) :transformation (send (elt frame-list-dest piv) :coord :rleg) :local) :rpy-angle))))
               (setq wyaw (car (car (send (send (elt frame-list piv) :coord :waist) :rpy-angle))))
               (send (send (elt tmp-list piv) :coord :waist) :rpy (+ wyaw on-yaw-diff) 0 0)
               ;; (send (elt tmp-list piv) :coord :rleg (send (elt frame-list-dest piv) :coord :rleg));; ずれた足を目標座標に戻す
               (warn "  ~a(~a)" piv :rleg)
               (incf piv)
               )
             )
            ;; (incf piv)
            )
          )
         (warn "~%after:~a~%" (rad2deg (car (car (send (send (send (elt tmp-list piv) :coord :rleg) :transformation (send (elt tmp-list piv) :coord :lleg) :local) :rpy-angle)))))
         ;; (setq piv (min lidx ridx));; (warn " ~a ~a~%" lidx ridx)
         )
       ;; 左足接地時
       ( (and (or= (send (elt frame-list piv) :state :lleg) (list *ground* *slide* *fan*)) (= (car (elt ik-fail-type-list piv)) *ik-yaw-fail-step*))
         (setq on-yaw-diff (car (car (send (send (send (elt frame-list piv) :coord :lleg) :transformation (send (elt frame-list-dest piv) :coord :lleg) :local) :rpy-angle))));; 新旧足先座標でのyaw-diff
         (setq wyaw (car (car (send (send (elt frame-list piv) :coord :waist) :rpy-angle))));; 腰yaw
         (send (send (elt tmp-list piv) :coord :waist) :rpy (+ wyaw on-yaw-diff) 0 0);; 足ずれた方と反対方向に腰を捻る
         ;; (send (elt tmp-list piv) :coord :lleg (send (elt frame-list-dest piv) :coord :lleg));; ずれた足を目標座標に戻す
         (warn "  ~a" piv)
         (incf piv)
         )
       ;; 右足接地時
       ( (and (or= (send (elt frame-list piv) :state :rleg) (list *ground* *slide* *fan*)) (= (cdr (elt ik-fail-type-list piv)) *ik-yaw-fail-step*))
         (setq on-yaw-diff (car (car (send (send (send (elt frame-list piv) :coord :rleg) :transformation (send (elt frame-list-dest piv) :coord :rleg) :local) :rpy-angle))))
         (setq wyaw (car (car (send (send (elt frame-list piv) :coord :waist) :rpy-angle))))
         (send (send (elt tmp-list piv) :coord :waist) :rpy (+ wyaw on-yaw-diff) 0 0)
         ;; (send (elt tmp-list piv) :coord :rleg (send (elt frame-list-dest piv) :coord :rleg));; ずれた足を目標座標にす戻す
         (warn "  ~a" piv)
         (incf piv)
         )
       ;; 非接地期
       ( t (incf piv) )
       )
      )

    (warn "~%")

    tmp-list
    ))

;; ik-fail足跡除去（水平位置）
;; rotも平均とってるけど無くすべき??
(defun detect-non-pos-ik-fail-footprint (frame-list ik-fail-num-list limb &key (floor 0))
  (let ( (piv 0) pos rpy idx term  denom step w init-flg ik-fail-count);; w 重率
    (while (< piv (length frame-list))
      (setq term 0   denom 0  pos #f(0 0 0)  rot (make-matrix 3 3) )
      (setq idx (eval piv))
      (setq init-flg t)

      (cond 
       ;; 完全接地のみ対象
       ( (= (send (elt frame-list idx) :state limb) *ground*)

         ;; 接地期のik-fail回数カウント    ik-failの割合にすべき？？
         (setq idx (eval piv) ik-fail-count 0)
         (while (and (< idx (length frame-list)) (= (send (elt frame-list idx) :state limb) *ground*) ) (incf ik-fail-count) (incf idx))

         ;; 平均値の算出
         (setq idx (eval piv))
         (while (and (< idx (length frame-list)) (= (send (elt frame-list idx) :state limb) *ground*) )
           ;; (cond ( (< ik-fail-count 10);; 同一支持脚中にik-fail数が少ない場合
           ;;         (cond
           ;;          ( (or (and init-flg (not (> (elt ik-fail-num-list idx) 0))) (> (elt ik-fail-num-list idx) 0))
           ;;            (setq w 1  step 1  init-flg nil) );; ik-fail時と最初のみ重率1
           ;;          ( t (setq w 0  step 0) ) ) )
           ;;       ( t;; 同一支持脚中にik-fail数が多い場合
           ;;         (cond ( (> (elt ik-fail-num-list idx) 0) (setq w 1  step 1));; ik-failは重率1
           ;;               (t (setq w 0.5  step 0.5))) );; それ以外は重率0.5
           ;;       )
           (setq w 1 step 1);; keypose

           (setq pos (v+ pos (scale w (send (elt frame-list idx) :pos limb))))
           (setq rot (m+ rot (scale-matrix w (send (elt frame-list idx) :rot limb))))
           (setq denom (+ denom step))
           (incf term)(incf idx) )
         (setf (elt pos 2) (* floor denom));; z成分は床高さ
         (setf (aref rot 2 2) denom);; yaw回り以外の回転はなし
         (setf (aref rot 2 0) 0)(setf (aref rot 2 1) 0)(setf (aref rot 0 2) 0)(setf (aref rot 1 2) 0)

         (dotimes (i term);; 平均値の代入
           (setq idx (+ piv i))
           (send (elt frame-list idx) :coord limb (make-coords :pos (scale (/ 1.0 denom) pos) :rot (scale-matrix (/ 1.0 denom) rot))))
         (setq piv (+ piv term)))

       ;; スライド,ファン,非接地
       ( t (incf piv))
       )

      );; end while

    frame-list
    ))

;; 滑りカット,足裏フィッティング
(defun foot-fit (frame-list limb &key (floor 0))
  (let ( (piv 0) pos rpy (idx 0) term)
    (while (< piv (length frame-list))
      (setq term 0   pos #f(0 0 0)  rot (make-matrix 3 3) )
      (setq idx (+ piv term))
      (cond 
       ;; 完全接地の場合
       ( (= (send (elt frame-list idx) :state limb) *ground*)
         (while (and (< idx (length frame-list)) (= (send (elt frame-list idx) :state limb) *ground*));; 接地中の平均値の算出
           (setq pos (v+ pos (send (elt frame-list idx) :pos limb)))
           (setq rot (m+ rot (send (elt frame-list idx) :rot limb)))
           (incf term)(incf idx) )
         (setf (elt pos 2) (* floor term));; z成分は床高さ
         (setf (aref rot 2 2) term);; yaw回り以外の回転はなし
         (setf (aref rot 2 0) 0)(setf (aref rot 2 1) 0)(setf (aref rot 0 2) 0)(setf (aref rot 1 2) 0)
         (dotimes (i term);; 平均値の代入
           (setq idx (+ piv i))
           (send (elt frame-list idx) :coord limb (make-coords :pos (scale (/ 1.0 term) pos) :rot (scale-matrix (/ 1.0 term) rot)) ))
         (setq piv (+ piv term)))
       ;; スライドとファンの場合
       ( (or (= (send (elt frame-list idx) :state limb) *slide*) (= (send (elt frame-list idx) :state limb) *fan*) )
         (setq pos (send (elt frame-list idx) :pos limb))
         (setq rot (send (elt frame-list idx) :rot limb))
         (setf (elt pos 2) floor);; z成分は床高さ
         (setf (aref rot 2 2) 1);; yaw回り以外の回転はなし
         (setf (aref rot 2 0) 0)(setf (aref rot 2 1) 0)(setf (aref rot 0 2) 0)(setf (aref rot 1 2) 0)
         (send (elt frame-list idx) :coord limb (make-coords :pos pos :rot rot) )
         (incf piv)
         )
       ;; 非接地の場合
       ( t (incf piv))
       )
      )
    frame-list))

;; 部分行列取り出し・代入
(defmethod array
  (:get-block
   (up left rows cols &optional (val-matrix nil))
   (let ( (ret-matrix (make-matrix rows cols)) )
     (dotimes (i rows)
       (setf (matrix-row ret-matrix i) (subseq (matrix-row self (+ up i)) left (+ left cols))) )
     ret-matrix
     )
   )
  (:set-block
   (up left val-matrix)
   (dotimes (i (val-matrix . dim0))
     (dotimes (j (val-matrix . dim1))
       (setf (aref self (+ up i) (+ left j)) (aref val-matrix i j))
       ))
   )
  )

;; kalman-filter
;; rpyはフィルタリングしていない
(defun av-kalman-filter ( input-list &key (r-variance 0.001) (q-variance 1e-4) )
  (let* ( (output-list (make-list (length input-list)))
          (measure-dim 1)
          ;; (state-dim 3)
          (state-dim 2)
          ;; (measure-dim (length (car input-list)))
          ;; (state-dim (* measure-dim 3))
          (A (unit-matrix state-dim))
          filter )
    (send A :set-block 0           measure-dim       (scale-matrix *ft* (unit-matrix measure-dim)))
    ;; (send A :set-block 0           (* measure-dim 2) (scale-matrix (* *ft* *ft* 0.5) (unit-matrix measure-dim))) 
    ;; (send A :set-block measure-dim (* measure-dim 2) (scale-matrix *ft* (unit-matrix measure-dim)))

    (dotimes (i (length (car input-list)))
      (cond
       ( (or (> i 5) (< i 3) )
         (setq filter (instance kalman-filter :init :measure-dim measure-dim :state-dim state-dim :r-variance r-variance :q-variance q-variance))
         (send filter :A A)
         (dotimes (frame-idx (length input-list))
           (push (elt (send filter :proc (float-vector (elt (elt input-list frame-idx) i))) 0) (elt output-list frame-idx)) )
         )
       ( t 
         (dotimes (frame-idx (length input-list))
           (push (elt (elt input-list frame-idx) i) (elt output-list frame-idx)) )
         )
       )
      )
    (mapcar #'(lambda (output) (coerce (reverse output) float-vector)) output-list )
    ;; (mapcar #'(lambda (input) (send filter :proc (elt input i))) input-list)
  ))

(defun vector-kalman-filter ( input-list target-slot &key (r-variance 0.001) (q-variance 1e-4) )
  (let* ( (output-list (copy-object input-list))
          (vector-list (make-list (length input-list)))
          (measure-dim 1)
          (state-dim 2)
          (A (unit-matrix state-dim))
          filter )
    (send A :set-block 0 measure-dim (scale-matrix *ft* (unit-matrix measure-dim)))

    (dotimes (i (length (send (car input-list) target-slot)))
      (setq filter (instance kalman-filter :init :measure-dim measure-dim :state-dim state-dim :r-variance r-variance :q-variance q-variance))
      (send filter :A A)
      (dotimes (frame-idx (length input-list))
        (push (elt (send filter :proc (subseq (send (elt input-list frame-idx) target-slot) i (1+ i))) 0) (elt vector-list frame-idx)) )
      )
    (dotimes (frame-idx (length input-list)) (send (elt output-list frame-idx) target-slot (coerce (reverse (elt vector-list frame-idx)) float-vector)) )
    output-list
  ))
(defun scalor-kalman-filter ( input-list target-slot &key (r-variance 0.001) (q-variance 1e-4) )
  (let* ( (output-list (copy-object input-list))
          (measure-dim 1)
          (state-dim 2)
          (A (unit-matrix state-dim))
          filter )
    (send A :set-block 0 measure-dim (scale-matrix *ft* (unit-matrix measure-dim)))

    (setq filter (instance kalman-filter :init :measure-dim measure-dim :state-dim state-dim :r-variance r-variance :q-variance q-variance))
    (send filter :A A)
    (dotimes (frame-idx (length input-list))
      (send (elt output-list frame-idx) target-slot (elt (send filter :proc (float-vector (send (elt input-list frame-idx) target-slot))) 0)) )
    output-list
    ))
(defun frame-kalman-filter ( input-list target-slot &key (r-variance 0.001) (q-variance 1e-4) )
  (let ( (target (send (car input-list) target-slot)) )
    (cond
     ( (vectorp target) (vector-kalman-filter input-list target-slot :r-variance r-variance :q-variance q-variance) )
     ( (numberp target) (scalor-kalman-filter input-list target-slot :r-variance r-variance :q-variance q-variance) )
     ( t (warning-message 1 "frame-kalman-filter does not support this type\n") )
     )
    ))
(defun do-kalman-filter ( input-list  &key (target-slot nil) (r-variance 0.001) (q-variance 1e-4) )
  (if target-slot
      (frame-kalman-filter input-list target-slot :r-variance r-variance :q-variance q-variance)
    (av-kalman-filter input-list :r-variance r-variance :q-variance q-variance) )
  )



;; LPF的な
(defun smoothfilter ( target-list &key (target-slot nil) (mod-list '(1 1 1 1 1)) (z-list nil) (state nil) )
  (let  ( (tmp-list (copy-object target-list) ) target )
    (dotimes (piv (length target-list))
      (let ( (diff  (/ (length mod-list) 2)) sum )
        ;; (when (and state (= piv 996)) (print (elt z-list piv)))

        (when (or (not state) (and z-list (= (elt z-list piv) state)))

          ;; frameリストかどうかで分岐
          (if target-slot
              (setq target (send (car target-list) target-slot))
            (setq target (car target-list)) )

          ;; 型違い対応
          (cond ( (floatp target) (setq sum 0)(setq func+ '+)(setq func* '*) );; float
                ( (vectorp target);; vector
                  (setq sum (make-array (length target) :element-type float-vector))
                  (setq func+ 'v+)(setq func* 'scale))
                ( (matrixp target);; matrix
                  (setq sum (make-matrix 3 3))
                  (setq func+ 'm+  func*  'scale-matrix) )
                ( (coordinates-p target);; coord
                  (setq sum (make-coords :rot (make-array '(3 3) :element-type float-vector :initial-element 0)))
                  (setq func+ 'coord+)(setq func* 'scale-coord))
                ( (and (consp target) (vectorp (car target)) );; joint-pos-list
                  (setq sum (make-list (length target) :initial-element #f(0 0 0)))
                  (setq func+ 'joint-pos-list+  func* 'scale-joint-pos-list) )
                ( t (warn "target type that smoothfilter dosen't support~%"))
                )


          ;; 平滑化処理
          (cond
           ( target-slot
             (dolist (mod mod-list)
               (if (and (> (+ piv diff) -1) (< (+ piv diff) (length target-list)))
                   (setq sum (funcall func+ sum (funcall func* mod (send (elt target-list (+ piv diff)) target-slot))))
                 (setq sum (funcall func+ sum (send (elt target-list piv) target-slot))))
               (decf diff))
             (send (elt tmp-list piv) target-slot (funcall func* (/ 1.0 (length mod-list)) sum) ) )
           ( t
             (dolist (mod mod-list)
               (if (and (> (+ piv diff) -1) (< (+ piv diff) (length target-list)))
                   (setq sum (funcall func+ sum (funcall func* mod (elt target-list (+ piv diff)))))
                 (setq sum (funcall func+ sum (elt target-list piv))))
               (decf diff))
             (setf (elt tmp-list piv) (funcall func* (/ 1.0 (length mod-list)) sum) ) )
           )


          );; end when
        ))

    tmp-list
    ))

;; 忍び足
;; 接地期間が１frameだと離陸時は処理できてない
;; 滑り期が短すぎると処理が微妙
;; (defun stealfilter (coord-list z-list &key (up-thresh 30) (down-thresh 30) (floor 0))
;;   (let ( tmp-list tmp-list1 (up-fnum (round (/ up-thresh (* 10 (/ *ft* 0.03))))) (down-fnum (round (/ down-thresh (* 10 (/ *ft* 0.03)))))
;;                   idx func1 func2 func3 func4 fnum thresh (min-thresh 5) (slide-fnum 7) )
;;     (setq tmp-list (copy-object coord-list)  tmp-list1 (copy-object z-list))
;;     (dotimes (piv (length coord-list));; リスト走査
;;       (block :low-step;; 足が閾値に達する前に下がった時のため
;;         (when (and (> piv 1) (< piv (1- (length coord-list))) (= (elt z-list piv) 0))

;;           (cond
;;            ( (not (= (elt z-list (1- piv)) 0));; 着地処理
;;              (setq upfunc '1-  downfunc '1+  upstep '-  downstep '+) (setq fnum down-fnum  thresh down-thresh) )
;;            ( (not (= (elt z-list (1+ piv)) 0));; 離陸処理
;;              (setq upfunc '1+  downfunc '1-  upstep '+  downstep '-) (setq fnum up-fnum  thresh up-thresh) )
;;            (t (return-from :low-step nil)) )

;;           (cond
;;            ( (or (= (elt z-list (1- piv)) 100) (= (elt z-list (1+ piv)) 100));; すり足の時
;;              (setq fnum (min slide-fnum (- (length coord-list) piv 1) piv))(setq idx (funcall upstep piv fnum))
;;              (setq thresh (elt (send (elt coord-list idx) :worldpos) 2)))
;;            (t (block :idx-move;; 遊脚振り上げ高さが低い時
;;                 (loop
;;                  (setq idx (funcall upfunc piv))
;;                  (while (and (not (= (elt z-list idx) 0)) (< (elt (send (elt coord-list idx) :worldpos) 2) thresh) );; idx移動
;;                    (setq idx (funcall upfunc idx))
;;                    (if (or (= idx 0) (= idx (length coord-list)) ) (return-from :low-step nil));; outof処理
;;                    );end while
;;                  (when (>= (elt (send (elt coord-list idx) :worldpos) 2) thresh) (return-from :idx-move nil))
;;                  (when (< thresh min-thresh) (return-from :low-step nil))
;;                  (setq thresh (/ thresh 2.0))(setq fnum (/ fnum 2));; 足高さが低い時のz-threshの低下
;;                  )));; end loop
;;            )

;;           (if (< (1- fnum) (abs (- piv idx)))
;;               (setq fnum (abs (- piv idx)));; 変化が元から緩やかなとき
;;             (if (> (funcall downstep idx fnum) (length coord-list))
;;                 (setq fnum (- (length coord-list) idx 1))
;;               (while (and (< (funcall downstep idx  fnum) (length coord-list))
;;                           (not (= (elt z-list (funcall downstep idx fnum)) 0)))
;;                 (decf fnum))));; 接地時間が短い際の衝撃緩和区間の短縮化

;;           (dotimes (i fnum);; z等比数列化
;;             (setf (elt (send (elt tmp-list idx) :worldpos) 2) (+ floor (* (- thresh floor) (expt 0.5 i))))
;;             (when (or (= piv 501) (= piv 490)) (warn "   ~a ~a" idx (* (- thresh floor) (expt 0.5 i))))
;;             (setq idx (funcall downfunc idx))
;;             )
;;           (when (or (= piv 501) (= piv 490)) (warn " ~%"))

;;           (dotimes (i fnum) (setf (elt tmp-list1 (funcall upstep idx (1+ i))) 30) );; 接地状態書き換え 衝撃緩和起動: 30
;;           (while (and (< idx (length coord-list)) (not (= (elt z-list idx) 0)))
;;             (setf (elt tmp-list1 idx) 0) (setq idx (funcall downfunc idx)))

;;           ));;block when end
;;       )

;;     (list tmp-list tmp-list1)
;;     ))

(defun stealfilter (frame-list limb &key (max-time 0.30) (min-time 0.12) (floor 0))
  (let ( tmp-list no-inpact-term (max-term (round (/ max-time *ft*))) (min-term (round (/ min-time *ft*))) range
                  idx start-idx end-idx local-max next-local-max  upfunc downfunc upstep downstep in edge )
    (setq tmp-list (copy-object frame-list))

    (dotimes (piv (length frame-list));; リスト走査
      (when (and (> piv 0) (< piv (1- (length frame-list)))
                 (or (= (send (elt frame-list piv) :state limb) *ground*) (= (send (elt frame-list piv) :state limb) *slide*) (= (send (elt frame-list piv) :state limb) *fan*) )
                 (or (and (not (= (send (elt frame-list (1+ piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1+ piv)) :state limb) *slide*))
                          (not (= (send (elt frame-list (1+ piv)) :state limb) *fan*)) )
                     (and (not (= (send (elt frame-list (1- piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1- piv)) :state limb) *slide*))
                          (not (= (send (elt frame-list (1- piv)) :state limb) *fan*)) ) ) );; 離着陸時のみ
        ;; (print piv)

        (cond
         ( (and (not (= (send (elt frame-list (1- piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1- piv)) :state limb) *slide*))
                (not (= (send (elt frame-list (1- piv)) :state limb) *fan*)) );; 着地処理
           (setq upfunc '1-  downfunc '1+  upstep '-  downstep '+  in '>=  edge 0 ) (setq fnum 10) )
         ( (and (not (= (send (elt frame-list (1+ piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1+ piv)) :state limb) *slide*))
                (not (= (send (elt frame-list (1+ piv)) :state limb) *fan*)) );; 離陸処理
           (setq upfunc '1+  downfunc '1-  upstep '+  downstep '-  in '<=  edge (1- (length frame-list)) ) (setq fnum 10) ) )

        ;; ;; 最近傍速度極大点調査
        ;; (setq local-max (eval piv)  next-local-max (funcall upfunc local-max))
        ;; (while (and (funcall in (funcall upfunc next-local-max) edge)  (> (send (elt frame-list next-local-max) :vel limb) (send (elt frame-list local-max) :vel limb)) )
        ;;   (setq local-max (eval next-local-max)  next-local-max (funcall upfunc next-local-max) )
        ;;   )

        ;; (setq no-inpact-term (min (1- (abs (- local-max piv))) max-term)
        ;;       no-inpact-term (max no-inpact-term min-term)
        ;;       end-idx (round (funcall upstep piv no-inpact-term))
        ;;       range (- (send (elt frame-list end-idx) :z limb) floor) )

        ;; (warn "~a ~a ~a ~a ~%" local-max no-inpact-term end-idx range)

        ;; キーポーズベースの時
        (setq start-idx (funcall upfunc piv) end-idx (eval piv))
        (while (and (funcall in (funcall upfunc end-idx) edge)  (not (member limb (send (elt frame-list (funcall upfunc end-idx)) :get-keypose-flg-list) ));; 対象肢体のキーポーズでない場合
                    (< (abs (- (send (elt frame-list piv) :time) (send (elt frame-list (funcall upfunc end-idx)) :time))) max-time) );; 速度極大は分からない
          (setq end-idx (funcall upfunc end-idx))
          )

        (setq idx (funcall upfunc piv))
        (while (and (<= (* (- idx start-idx) (- idx end-idx)) 0) (funcall in start-idx end-idx))
          ;; (setf (elt coord-list-tmp idx) (make-coords :pos (concatenate float-vector (subseq (send (elt coord-list-tmp idx) :worldpos) 0 2) 
          ;;                                                               (float-vector (+ floor (* range (expt 0.5 (abs (- idx end-idx)))))) )
          ;;                                             :rot (send (elt coord-list-tmp idx) :worldrot) ) );; z等比数化列
          (send (elt tmp-list idx) :state limb *non-inpact*);; 接地状態書き換え 衝撃緩和軌道: 30
          (setq idx (funcall upfunc idx))
          )

        )
      );; end dotimes

    tmp-list
    ))

;; シグモイド関数
(defun sigmoid (x &key (gain 1) (thresh 0) ) (/ 1.0 (+ 1 (exp (* gain -1 (- x thresh))))) )

;; 垂直接地
(defun verticalfilter (frame-list limb)
  (let* ( tmp-list term  rot relrot pivrot pos relpos pivpos upfunc downfunc upstep idx end-idx neighbor local-max in edge
                  (min-time 0.15) (min-term (round (/ min-time *ft*))) (max-time 0.5) (max-term (round (/ max-time *ft*))) )
    (setq tmp-list (copy-object frame-list))

    (dotimes (piv (length frame-list));; リスト走査
      (block :vertical
        (when (and (> piv 1) (< piv (1- (length frame-list))) 
                   (or (= (send (elt frame-list piv) :state limb) *ground*) (= (send (elt frame-list piv) :state limb) *slide*) (= (send (elt frame-list piv) :state limb) *fan*) )
                   (or (and (not (= (send (elt frame-list (1+ piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1+ piv)) :state limb) *slide*))
                            (not (= (send (elt frame-list (1+ piv)) :state limb) *fan*)) )
                       (and (not (= (send (elt frame-list (1- piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1+ piv)) :state limb) *slide*))
                            (not (= (send (elt frame-list (1+ piv)) :state limb) *fan*)) ) ) );; 接地前後のみ
          (setq pivpos (send (elt tmp-list piv) :pos limb)) (setq pivrot (send (elt tmp-list piv) :rot limb))

          (dotimes (turn 2)
            (cond
             ( (and (not (= (send (elt frame-list (1- piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1- piv)) :state limb) *slide*)) (not (= (send (elt frame-list (1- piv)) :state limb) *fan*)) 
                    (= turn 0));; 着地処理
               (setq upfunc '1-  downfunc '1+  upstep '-  in '>=  edge 0 )
               (setq neighbor (send (elt frame-list (1- piv)) :state limb)) )
             ( (or (and (not (= (send (elt frame-list (1+ piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1+ piv)) :state limb) *slide*))
                        (not (= (send (elt frame-list (1+ piv)) :state limb) *fan*))
                        (= turn 0));; 離陸処理
                   (and (not (= (send (elt frame-list (1+ piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1+ piv)) :state limb) *slide*)) 
                        (not (= (send (elt frame-list (1+ piv)) :state limb) *fan*))
                        (not (= (send (elt frame-list (1- piv)) :state limb) *ground*)) (not (= (send (elt frame-list (1- piv)) :state limb) *slide*))
                        (not (= (send (elt frame-list (1- piv)) :state limb) *fan*)) ) );; 離着陸処理
               (setq upfunc '1+  downfunc '1-  upstep '+  in '<=  edge (1- (length frame-list)) )
               (setq neighbor (send (elt frame-list (1+ piv)) :state limb)) )
             ( t (return-from :vertical nil) ) )

            ;; ;; 最近傍速度極大点による区間調査
            ;; (setq idx (funcall upfunc piv))(setq term 1)
            ;; (while (and (>= (funcall upfunc idx) 0) (<= (funcall upfunc idx) (1- (length frame-list)))
            ;;             (= (send (elt frame-list (funcall upfunc idx)) :state limb) neighbor) )
            ;;   (setq idx (funcall upfunc idx)) (incf term)  )

            ;; ;; 最近傍高さ極大点調査
            ;; (setq local-max (eval piv)  next-local-max (funcall upfunc local-max))
            ;; (while (and (funcall in (funcall upfunc next-local-max) edge)
            ;;             (or (> (send (elt frame-list next-local-max) :z limb) (send (elt frame-list local-max) :z limb) )
            ;;                 (< (send (elt frame-list local-max) :z limb) *bottom*) ))
            ;;   (setq local-max (eval next-local-max)  next-local-max (funcall upfunc next-local-max) )
            ;;   )

            ;; ;; 高さ極大点と速度極大点から垂直接地区間を決定
            ;; (setq term (max (min (1- (abs (- local-max piv))) max-term) term);; 遠い方をend-idxにする
            ;;       ;; term (max no-inpact-term min-term)
            ;;       end-idx (round (funcall upstep piv term)) )

            ;; ;; ;; 卒論のため最近傍速度極大点のみ考慮
            ;; ;; (setq end-idx (round (funcall upstep piv term)))

            ;; キーポーズベースの場合
            (setq end-idx (eval piv))
            (while (and (funcall in (funcall upfunc end-idx) edge)  (equal (send (elt frame-list (funcall upfunc end-idx)) :state limb) neighbor) )
              (setq end-idx (funcall upfunc end-idx))
              )


            ;; 垂直接地軌道 代入
            ;; (setq pos (send (elt tmp-list end-idx) :worldpos)) (setq relpos (v- pos pivpos))
            ;; (setq rot (send (elt tmp-list end-idx) :worldrot)) (setq relrot (m- rot pivrot))
            (setq idx (funcall upfunc piv))
            (while (and (< (* (- idx piv) (- idx end-idx)) 0) (funcall in (funcall upfunc piv) end-idx) )
              (setq pos (send (elt tmp-list idx) :pos limb)) (setq relpos (v- pos pivpos));; tmp-listでいい？？frame-listでなくていい？？
              (setq rot (send (elt tmp-list idx) :rot limb)) (setq relrot (m- rot pivrot))

              ;; pos
              ;; (setq pos (v+ pivpos (scale (sigmoid (/ (abs (- idx piv)) (float term)) :gain 10 :thresh 0.5 ) relpos)))
              ;; (setq pos (v+ pivpos (scale (sigmoid (/ (abs (- idx piv)) (float term)) :gain 7 :thresh 0.5 ) relpos)))
              (setq pos (v+ pivpos (scale (sigmoid (/ (abs (- (send (elt frame-list idx) :time) (send (elt frame-list piv) :time))) max-time) :gain 7 :thresh 0.5 ) relpos)))
              ;; (setq pos (v+ pivpos (scale (/ (abs (- idx piv)) (float term) )  relpos)))
              ;; (setq pos (v+ pivpos (scale (expt 0.5 (abs (- idx end-idx))) relpos)))
              ;; (setq pos (v+ pivpos (scale (expt 0.75 (abs (- idx end-idx))) relpos)))
              ;; z
              ;; (setf (elt pos 2) (elt (send (elt coord-list idx) :worldpos) 2))
              ;; rot
              ;; (setq rot (m+ pivrot (scale-matrix (sigmoid (/ (abs (- idx piv)) (float term)) :gain 10 :thresh 0.5 ) relrot)))
              (setq rot (m+ pivrot (scale-matrix (sigmoid (/ (abs (- (send (elt frame-list idx) :time) (send (elt frame-list piv) :time))) max-time) :gain 10 :thresh 0.5 ) relrot)))
              ;; (setq rot (m+ pivrot (scale-matrix (/ (abs (- idx piv)) (float term)) relrot)))
              ;; (setq rot (m+ pivrot (scale-matrix (expt 0.5 (abs (- idx end-idx))) relrot)))
              ;; (setq rot (m+ pivrot (scale-matrix (expt 0.75 (abs (- idx end-idx))) relrot)))
              (send (elt tmp-list idx) :coord limb (make-coords :pos pos  :rot rot));; 座標更新
              (setq idx (funcall upfunc idx))
              );; (warn "~a ~a ~a~%" piv local-max end-idx)

            );; turn end
          ));;block when end
      )
    tmp-list
    ))

;; 目標z座標 接地面以下除去
(setq *bottom* 10)
(defun raise-bottom (frame-list limb)
  (let* ( tmp-list  (top 100)  min (range  (float (- top *bottom*))) idx (piv 0) pos ) ;; 目標z座標は遊脚期では10mm以上
    (setq tmp-list (copy-object frame-list))
    (while (< piv (length frame-list));; リスト走査

      (when (< (send (elt frame-list piv) :z limb) top)
        (setq min *bottom*  idx (eval piv))

        (while (and (< idx (length frame-list)) (< (send (elt frame-list idx) :z limb) top));; min探索
          (if (and (< (send (elt frame-list idx) :z limb) min)
                   (not (= (send (elt frame-list idx) :state limb) *ground*)) (not (= (send (elt frame-list idx) :state limb) *slide*)) (not (= (send (elt frame-list idx) :state limb) *fan*)))
              (setq min (send (elt frame-list idx) :z limb)))
          (incf idx))

        (when (< min *bottom*)
          (setq  idx (eval piv))
          (while (and (< idx (length frame-list)) (< (send (elt frame-list idx) :z limb) top))
            ;; (setf (elt (send (elt tmp-list idx) :worldpos) 2) (+ *bottom* (* (/ range (- top min)) (- (elt (send (elt coord-list idx) :worldpos) 2) min))))
            (setq pos (send (elt frame-list idx) :pos limb))
            (send (elt tmp-list idx) :pos limb (float-vector (elt pos 0) (elt pos 1) (+ *bottom* (* (/ range (- top min)) (- (elt pos 2) min))) ))
            (incf idx))
          (setq piv (eval idx)))

        );; end when

      (incf piv)
      )
    tmp-list
    ))

;; posから速さを計算
(defun pos2vel (pos-list)
  (let ( (vel-list nil) )
    (dotimes (piv (length pos-list))
      (cond
       ((and (> piv 0) (< piv (1- (length pos-list))))
        (push (/ (norm (v- (elt pos-list (1+ piv)) (elt pos-list (1- piv)))) 2.0) vel-list))
       ((= piv 0)(push (norm (v- (elt pos-list (1+ piv)) (elt pos-list piv))) vel-list))
       (t (push (norm (v- (elt pos-list  piv) (elt pos-list (1- piv)))) vel-list)))
      )
    (reverse vel-list)
    ))
(defun calc-vel-from-pos (frame-list limb)
  (let ( (tmp-list (copy-object frame-list)) )
    (dotimes (piv (length frame-list))
      (cond
       ( (and (> piv 0) (< piv (1- (length frame-list))))
         (send (elt tmp-list piv) :vel limb (/ (norm (v- (send (elt frame-list (1+ piv)) :pos limb)  (send (elt frame-list (1- piv)) :pos limb) )) 2.0)) )
       ( (= piv 0)
         (send (elt tmp-list piv) :vel limb (norm (v- (send (elt frame-list (1+ piv)) :pos limb) (send (elt frame-list piv) :pos limb)))) )
       (t (send (elt tmp-list piv) :vel limb (norm (v- (send (elt frame-list  piv) :pos limb) (send (elt frame-list (1- piv)) :pos limb)))) ))
      )
    tmp-list
    ))

;; 速さから加速度の大きさを計算
;; 使っていない???
(defun vel2acc (vel-list)
  (let ( (acc-list nil) prev next )
    (dotimes (piv (length vel-list))
      (setq prev (max 0 (1- piv)))(setq next (min (1+ piv) (1- (length vel-list))))
      ;; (push (norm (v+ (elt pos-list next) (scale -2 (elt pos-list piv)) (elt pos-list prev))) acc-list)
      (push (/ (- (elt vel-list next) (elt vel-list prev)) 2.0) acc-list)
      )
    (reverse acc-list)
    ))

;; 姿勢からヨー軸周り角速度の大きさを計算
(defun rot2yawvel (rot-list)
  (let ( (yawvel-list nil) )
    (dotimes (piv (length rot-list))
      (cond
       ( (and (> piv 0) (< piv (1- (length rot-list))))
         (push (abs (aref (m* (scale-matrix 0.5 (m- (elt rot-list (1+ piv)) (elt rot-list (1- piv)))) (transpose (elt rot-list piv))) 0 1)) yawvel-list) )
       ( (= piv 0)
         (push (abs (aref (m* (m- (elt rot-list (1+ piv)) (elt rot-list piv)) (transpose (elt rot-list piv))) 0 1)) yawvel-list) )
       ( t
         (push (abs (aref (m* (m- (elt rot-list piv) (elt rot-list (1- piv))) (transpose (elt rot-list piv))) 0 1)) yawvel-list)) )
      )
    (reverse yawvel-list)
    ))
(defun calc-yawvel-from-rot (frame-list limb)
  (let ( (tmp-list (copy-object frame-list)) )
    (dotimes (piv (length frame-list))
      (cond
       ( (and (> piv 0) (< piv (1- (length frame-list))))
         (send (elt tmp-list piv) :yawvel limb (abs (aref (m* (scale-matrix 0.5 (m- (send (elt frame-list (1+ piv)) :rot limb) (send (elt frame-list (1- piv)) :rot limb)))
                                                              (transpose (send (elt frame-list piv) :rot limb))) 0 1))) )
       ( (= piv 0)
         (send (elt tmp-list piv) :yawvel limb (abs (aref (m* (m- (send (elt frame-list (1+ piv)) :rot limb) (send (elt frame-list piv) :rot limb)) 
                                                              (transpose (send (elt frame-list piv) :rot limb))) 0 1))) )
       ( t
         (send (elt tmp-list piv) :yawvel limb (abs (aref (m* (m- (send (elt frame-list piv) :rot limb) (send (elt frame-list (1- piv)) :rot limb)) 
                                                              (transpose (send (elt frame-list piv) :rot limb))) 0 1)))) )
      )
    tmp-list
    ))




;; 接地状態判定 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 足面のピッチ角度を計算 足面の水平面との角度計算
(defun calc-pitch (val)
  (let (x y_ x_)
    (cond
     ( (coordinates-p val) (setq x (normalize-vector (matrix-column (send val :rot)  0))) );; 足先座標から求めるとき
     ( (matrixp val)       (setq x (normalize-vector (matrix-column val 0))) );; 足先姿勢から求めるとき
     ( (vectorp val) ;; 足ベクトルから求めるとき
       (setq x (normalize-vector val) y_ (v* #f(0 0 1) x)
             x (transform (rotation-matrix (* -1 (deg2rad *pitch*)) y_) x) ) ) ;; vecの時だけ*pitch*分回す
     )
    (setq y_ (v* #f(0 0 1) x))
    (setq x_ (v* y_ #f(0 0 1)))
    (acos (v. x_ x));; [rad]を返す
    ))

;; 足ベクトルのみから足先座標計算
;;;;;;; 回帰平面は用いない
;;;;;;; *pitch*分回しておく
(defun calc-rot (foot-vec)
  (let (x y z rot)
    (setq x (normalize-vector foot-vec)
          y (normalize-vector (v* #f(0 0 1) x))
          x (transform (rotation-matrix (* -1 (deg2rad *pitch*)) y) x);; *pitch*分回転
          z (normalize-vector (v* x y)));; z計算
    (setq rot (transpose (matrix x y z)))

    ;; (rotate-matrix rot (* -1 (deg2rad *pitch*)) :y);; *pitch*分回転
    rot
    ))

;; 接地、スライド、ヒール・トーポイント、ファン検出
 ;; 0:完全接地 20:タッチ 50:ポイント  100:滑り 120:ドロー   70:ファン  200:非接地   10000:未定
(setq *ground* 0 *touch* 20 *point* 50   *slide* 100 *draw* 120    *fan* 70    *float* 200  *undefined* 10000  *non-inpact* 30)
(defun check-foot-state (frame-list &key (z-thresh 100) (vel-thresh 10) (pos-thresh 100) (yawvel-thresh 0.05) (yawangle-thresh 1)
                                    (point-z-thresh 150) (point-angle-thresh 20) (point-vel-thresh 20) (touch-thresh 0.25) (draw-thresh 0.35) (term 4) )
  (let ()

    (warn "z-thresh ~a  vel-thresh ~a  pos-thresh ~a~%point-angle-thresh ~a  point-z-thresh ~a point-vel-thresh ~a~% yawvel-thresh ~a  yawangle-thresh ~a~%touch-thresh ~a  draw-thresh ~a~% floor ~a ~%"
          z-thresh vel-thresh pos-thresh point-angle-thresh point-z-thresh point-vel-thresh yawvel-thresh yawangle-thresh touch-thresh draw-thresh floor)

    ;; スライド・ドロー識別
    (setq mframe-list (recognize-slide mframe-list :lleg :vel-thresh vel-thresh :pos-thresh pos-thresh :z-thresh z-thresh)
          mframe-list (recognize-slide mframe-list :rleg :vel-thresh vel-thresh :pos-thresh pos-thresh :z-thresh z-thresh) )

    ;; ファン識別
    (setq mframe-list (recognize-fan mframe-list :lleg :yawvel-thresh yawvel-thresh :yawangle-thresh yawangle-thresh :z-thresh z-thresh)
          mframe-list (recognize-fan mframe-list :rleg :yawvel-thresh yawvel-thresh :yawangle-thresh yawangle-thresh :z-thresh z-thresh) )

    ;; 接地判定
    (setq mframe-list (recognize-static-foot-state mframe-list :lleg
                                        :z-thresh z-thresh :vel-thresh vel-thresh :point-z-thresh point-z-thresh :point-angle-thresh point-angle-thresh :point-vel-thresh point-vel-thresh)
          mframe-list (recognize-static-foot-state mframe-list :rleg
                                        :z-thresh z-thresh :vel-thresh vel-thresh :point-z-thresh point-z-thresh :point-angle-thresh point-angle-thresh :point-vel-thresh point-vel-thresh))
    (warn "end foot state~%")

    ;; (setq lstate-list-tmp nil)(dolist (val mframe-list) (push-back (send val :state :lleg) lstate-list-tmp))
    ;; (setq rstate-list-tmp nil)(dolist (val mframe-list) (push-back (send val :state :rleg) rstate-list-tmp))

    ;; 前後フィルタ
    (setq mframe-list (min-term mframe-list :lleg :term term) mframe-list (min-term mframe-list :rleg :term term))
    (warn "end min term~%")

    ;; (setq lstate-list-tmp1 nil)(dolist (val mframe-list) (push-back (send val :state :lleg) lstate-list-tmp1))
    ;; (setq rstate-list-tmp1 nil)(dolist (val mframe-list) (push-back (send val :state :rleg) rstate-list-tmp1))

    ;; タッチ識別
    (setq mframe-list (separate-touch-ground mframe-list :lleg :thresh touch-thresh)
          mframe-list (separate-touch-ground mframe-list :rleg :thresh touch-thresh))

    ;; ドロー識別
    (setq mframe-list (separate-draw-slide mframe-list :lleg :thresh draw-thresh)
          mframe-list (separate-draw-slide mframe-list :rleg :thresh draw-thresh))

    mframe-list
    ))

;; 面接地、ポイント、非接地識別
(defun recognize-static-foot-state (frame-list limb &key (z-thresh 100) (vel-thresh 10)  (point-z-thresh 150) (point-angle-thresh 20) (point-vel-thresh 20) )
  (let ( (tmp-list (copy-object frame-list))  theta pre-state)
    (dotimes (piv (length frame-list))
      (setq pre-state (send (elt tmp-list (max (1- piv) 0)) :state limb));; 前状態記憶

      (cond
       ;; 滑り動作
       ( (= (send (elt frame-list piv) :state limb) *fan*) );; ファン
       ( (= (send (elt frame-list piv) :state limb) *slide*) );; スライド・ドロー

       ;; 停止動作
       ;; ヒール・トーポイント
       ( (and (< (send (elt frame-list piv) :z limb) point-z-thresh)
              (< (send (elt frame-list piv) :vel limb) point-vel-thresh) ;; z-thresh < z < point-z-threshの時のために必要
              (not (eps= (rad2deg (calc-pitch (send (elt frame-list piv) :coord limb))) 0 point-angle-thresh))
              (not (= pre-state *ground*));; 前状態が面接地だとポイントと見なさない
              )
         (send (elt tmp-list piv) :set-state limb *point*) )
       ;; 面接地 (ここではタッチも含む)
       ( (< (send (elt frame-list piv) :z limb) z-thresh)
         (send (elt tmp-list piv) :set-state limb *ground*) )

       ;; 非接地
       ( t 
         (send (elt tmp-list piv) :set-state limb *float*) )
       )
      )
    tmp-list
    ))

;; タッチ識別
(defun separate-touch-ground (frame-list touch-limb &key (thresh 0.25))
  (let (s0 e0 s1 e1 zmp-avg idx pos-touch pos-sub rate sub-limb)
    (warn "touch-ground rate~%")
    (cond ((equal touch-limb :lleg) (setq sub-limb :rleg)) ((equal touch-limb :rleg) (setq sub-limb :lleg)))
    (setq touch-rate-list nil)
    (dotimes (piv (length frame-list))
      (when (and (= (send (elt frame-list piv) :state touch-limb) *ground*) (= (send (elt frame-list piv) :state sub-limb) *ground*))

        ;; 接地区間探索
        (setq s0 (eval piv)  s1 (eval piv) e0 (eval piv) e1 (eval piv))
        (while (and (<= (1+ e0) (1- (length frame-list))) (= (send (elt frame-list (1+ e0)) :state touch-limb) *ground*)) (incf e0))
        (while (and (>= (1- s1) 0)                        (= (send (elt frame-list (1- s1)) :state sub-limb)   *ground*)) (decf s1))
        (while (and (<= (1+ e1) (1- (length frame-list))) (= (send (elt frame-list (1+ e1)) :state sub-limb)   *ground*)) (incf e1))

        (when (and (< s1 s0) (< e0 e1))

          ;; 平均 zmp 足位置
          (setq zmp-avg #f(0 0 0) pos-touch #f(0 0 0)  pos-sub #f(0 0 0)  idx (eval s0))
          (while (<= idx e0)
            (setq zmp-avg (v+ zmp-avg (send (elt frame-list idx) :zmp))  pos-touch (v+ pos-touch (send (elt frame-list idx) :pos touch-limb)) pos-sub (v+ pos-sub (send (elt frame-list idx) :pos sub-limb)))
            (incf idx) )
          (setq zmp-avg (scale (/ 1.0 (1+ (- e0 s0))) zmp-avg) pos-touch (scale (/ 1.0 (1+ (- e0 s0))) pos-touch) pos-sub (scale (/ 1.0 (1+ (- e0 s0))) pos-sub) )
          (setf (elt pos-touch 2) 0)(setf (elt pos-sub 2) 0)


          (setq rate (/ (norm (v- zmp-avg pos-sub)) (norm (v- pos-touch pos-sub))))
          (push-back rate touch-rate-list)

          ;; タッチ判定
          (warn " ~a:" s0)
          (cond
           ( (< rate thresh)
             (warning-message *touch-color-num* " ~a" rate);; タッチ
             ;; (warn " ~a ~a" (v- zmp-avg pos-sub) (v- pos-touch pos-sub))
             (setq idx (eval s0))
             (while (<= idx e0) (send (elt frame-list idx) :state touch-limb *touch*) (incf idx)) )
           ( t (warn " ~a" rate) );; 面接地
           )

          )
        (setq piv (1+ e0))
        )
      );; end dotimes
    (warn "~%")

    frame-list
    ))

;; ドロー識別
(defun separate-draw-slide (frame-list draw-limb &key (thresh 0.35))
  (let (s0 e0 s1 e1 zmp-avg idx pos-touch pos-sub margine rate max-rate min-rate sub-limb)
    (warn "draw-slide rate~%")
    (cond ((equal draw-limb :lleg) (setq sub-limb :rleg)) ((equal draw-limb :rleg) (setq sub-limb :lleg)))
    (setq draw-rate-list nil)
    (dotimes (piv (length frame-list))
      (when (and (= (send (elt frame-list piv) :state draw-limb) *slide*) (not (= (send (elt frame-list piv) :state sub-limb) *slide*)));; ドロー・スライドの組み合わせはない？？

        ;; スライド区間探索
        (setq s0 (eval piv)  s1 (eval piv) e0 (eval piv) e1 (eval piv))
        (while (and (<= (1+ e0) (1- (length frame-list))) (= (send (elt frame-list (1+ e0)) :state draw-limb) *slide*)) (incf e0))
        ;; (while (and (>= (1- s1) 0)                          (= (elt z-list-sub   (1- s1)) *ground*)) (decf s1))
        ;; (while (and (<= (1+ e1) (1- (length z-list-draw))) (= (elt z-list-sub   (1+ e1)) *ground*)) (incf e1))
        (setq margine (round (* 0.2 (- e0 s1))))

        ;; (when (and (< s1 s0) (< e0 e1))

        ;; zmpと足位置の距離比の平均計算(距離の平均は求めない)
        ;; (setq rate 0 idx (eval s0))
        ;; (setq rate 0 idx (+ s0 margine))
        (setq max-rate 0 min-rate 1 idx (+ s0 margine))
        (while (<= idx (- e0  margine))
          (setq pos-touch (send (elt frame-list idx) :pos draw-limb)  pos-sub (send (elt frame-list idx) :pos sub-limb))
          (setf (elt pos-touch 2) 0)(setf (elt pos-sub 2) 0);; ２次元化 床面へ投影
          ;; (setq rate (+ rate (/ (norm (v- (elt zmp-list idx) pos-sub)) (norm (v- pos-touch pos-sub))))) (incf idx)

          ;; (setq max-rate (max max-rate (/ (norm (v- (elt zmp-list idx) pos-sub)) (norm (v- pos-touch pos-sub)))))
          (setq min-rate (min min-rate (/ (norm (v- (send (elt frame-list idx) :zmp) pos-sub)) (norm (v- pos-touch pos-sub)))))
          (incf idx)
          )
        ;; (setq rate (- max-rate min-rate))
        ;; (setq rate (/ rate (1+ (- e0 s0))) )

        ;; rate リスト記録
        (setq idx (eval s0))
        (while (<= idx e0)
          (setq pos-touch (send (elt frame-list idx) :pos draw-limb)  pos-sub (send (elt frame-list idx) :pos sub-limb))
          (setf (elt pos-touch 2) 0)(setf (elt pos-sub 2) 0);; ２次元化 床面へ投影
          (push-back (/ (norm (v- (send (elt frame-list idx) :zmp) pos-sub)) (norm (v- pos-touch pos-sub))) draw-rate-list) (incf idx)
          )

        ;; ドロー判定
        (warn "  ~a:" s0)
        (cond
         ;; ( (or (< rate thresh) (not (= (elt z-list-draw (max (1- s0) 0)) *ground*)) (not (= (elt z-list-draw (min (1+ e0) (1- (length z-list-draw)))) *ground*)) )
         ( (< min-rate thresh) ;; 閾値決定用
           (warning-message *draw-color-num* " ~a" min-rate);; ドロー
          (setq idx (eval s0))
          (while (<= idx e0) (send (elt frame-list idx) :state draw-limb *draw*) (incf idx)) )
         (t (warning-message *slide-color-num* " ~a" min-rate) );; スライド 両端が面接地でなければならない
         )

        ;; )
        (setq piv (1+ e0))
        )

      (when (< piv (length frame-list))
        (setq pos-touch (send (elt frame-list piv) :pos draw-limb)  pos-sub (send (elt frame-list piv) :pos sub-limb))
        (setf (elt pos-touch 2) 0)(setf (elt pos-sub 2) 0);; ２次元化 床面へ投影
        (push-back (/ (norm (v- (send (elt frame-list piv) :zmp) pos-sub)) (norm (v- pos-touch pos-sub))) draw-rate-list) )

      );; end dotimes
    (warn "~%")

    frame-list
    ))

;; ファン識別
(defun recognize-fan (frame-list limb &key (yawvel-thresh 0.05) (z-thresh 100) (yawangle-thresh 1) )
  (let (s e idx integral tmp-list)
    (setq tmp-list (copy-object frame-list))
    (warn "yaw angle integral~%")
    (dotimes (piv (length frame-list))
      (when (and (> (send (elt frame-list piv) :yawvel limb) yawvel-thresh) (< (send (elt frame-list piv) :z limb) z-thresh) )
        (setq integral 0)

        ;; ファン区間探索
        (setq s (eval piv)  e (eval piv))
        (while (and (<= (1+ e) (1- (length frame-list))) (> (send (elt frame-list (1+ e)) :yawvel limb) yawvel-thresh) (< (send (elt frame-list (1+ e)) :z limb) z-thresh) )
          (incf e)
          (setq integral (+ integral (send (elt frame-list e) :yawvel limb)))
          )

        (warn "  ~a:" piv)
        (cond
         ( (> integral yawangle-thresh)
           (setq idx (eval s))
           (warning-message *fan-color-num* " ~a" integral)
           (while (<= idx e)
             (send (elt tmp-list idx) :set-state limb *fan*) (incf idx) ) )
         ( (warn " ~a" integral) )
         )

        (setq piv (eval (1+ e)))
        )
      );; end dotimes
    (warn "~%")

    tmp-list
    ))

;; スライド・ドロー識別
(defun recognize-slide (frame-list limb &key (vel-thresh 0.05) (z-thresh 100) (pos-thresh 100) )
  (let (s e idx integral tmp-list)
    (setq tmp-list (copy-object frame-list))
    (warn "vel integral~%")
    (dotimes (piv (length frame-list))
      (when (and (> (send (elt frame-list piv) :vel limb) vel-thresh) (< (send (elt frame-list piv) :z limb) z-thresh) )
        (setq integral 0)

        ;; スライド区間探索
        (setq s (eval piv)  e (eval piv))
        (while (and (<= (1+ e) (1- (length frame-list))) (> (send (elt frame-list (1+ e)) :vel limb) vel-thresh) (< (send (elt frame-list (1+ e)) :z limb) z-thresh) )
          (incf e)
          (setq integral (+ integral (send (elt frame-list e) :vel limb)))
          )

        (warn "  ~a:" piv)
        (cond
         ( (> integral pos-thresh)
           (setq idx (eval s))
           (warning-message *slide-color-num* " ~a" integral)
           (while (<= idx e)
             (send (elt tmp-list idx) :set-state limb *slide*) (incf idx) ) );; とりあえずスライドと識別しておく
         ( (warn " ~a" integral) )
         )

        (setq piv (eval (1+ e)))
        )
      );; end dotimes
    (warn "~%")

    tmp-list
    ))


;; 継続期間の短い状態を除去
(defun min-term (frame-list limb &key (term 2) );; term 最短継続期間
  (let ( end-idx idx  state-list count state prev-state next-state (tmp-list (copy-object frame-list)) );; 接地判定直後は100も変更されてない

    ;; 状態リスト作成 高さ値から非接地状態変数への変換
    ;; (setq state-list (copy-object target))
    ;; (dotimes (piv (length frame-list))
    ;;   (if (and (not (= (send (elt frame-list piv) :state limb) *ground*)) (not (= (send (elt frame-list piv) :state limb) *point*))
    ;;            (not (= (send (elt frame-list piv) :state limb) *slide*)) (not (= (send (elt frame-list piv) :state limb) *fan*)) )
    ;;       (send (elt state-list piv) *float*) ) )

    ;; 未定状態代入
    (dotimes (piv (length frame-list));; リスト走査
      (setq count 1  idx (eval piv));; 連続数計算
      (while (and (>= (1- idx) 0) (= (send (elt frame-list piv) :state limb) (send (elt frame-list (1- idx)) :state limb)) ) (decf idx) (incf count))
      (setq idx (eval piv))
      (while (and (<= (1+ idx) (1- (length frame-list))) (= (send (elt frame-list piv) :state limb) (send (elt frame-list (1+ idx)) :state limb))) (incf idx) (incf count))

      (if (< count term) (send (elt tmp-list piv) :state limb *undefined*) )
      )

    (dotimes (piv (length frame-list))
      (when (= (send (elt tmp-list piv) :state limb) *undefined*);; 未定状態の時
        (setq count 1  end-idx (eval piv));; 未定状態連続数計算 end-idx決定
        (while (and (<= (1+ end-idx) (1- (length tmp-list))) (= (send (elt tmp-list (1+ end-idx)) :state limb) *undefined*)) (incf end-idx) (incf count))

        ;; 未定状態の割り振り
        (setq prev-state (send (elt frame-list (max 0 (1- piv))) :state limb)  next-state (send (elt frame-list (min (1+ end-idx) (1- (length frame-list)))) :state limb) )
        (setq state (max prev-state next-state) );; minにした方がjumpが少なくなる maxにすると接地前後の挙動が正しくなる
        ;; (warn "~a ~a ~a~%" piv end-idx state)

        ;; 未定状態代入
        (setq idx (eval piv))
        (while (<= idx end-idx) (send (elt tmp-list idx) :state limb state) (incf idx))
        (setq piv (min (1+ end-idx) (1- (length frame-list))))
        )
      )

    ;; ;; 非接地状態変数から高さ値への変換
    ;; (dotimes (piv (length target))
    ;;   (if (= (elt state-list piv) *float*)
    ;;       ;; (if (and (not (= (elt target piv) *ground*)) (not (= (elt target piv) *point*)) (not (= (elt target piv) *slide*)) )
    ;;       ;;     (setf (elt state-list piv) (elt target piv))
    ;;       ;;   (setf (elt state-list piv) (elt target (max 0 (1- piv)) ) ) )
    ;;       (setf (elt state-list piv) (elt z-list-origin piv))
    ;;     )
    ;;   )

    tmp-list
    ))


;; 完全遊脚軌道生成 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; 遊脚軌道 地面食い込み ik-fail 自己干渉 除去
(defun decide-free-leg (frame-list free-limb &key (animate nil))
  (let ( (cut-in-flg t)  (ik-fail-flg t)  (interfere-flg nil) tmp-list return-list (loopnum 0) cut-in-list ik-fail-list interfere-list )
    (setq tmp-list (copy-object frame-list))

    (warn "~%decide free leg ~a~%"  free-limb)

    ;; 地面食い込み傾き除去
    (setq tmp-list (avoid-orient-cut-in tmp-list free-limb))

    (while (or interfere-flg ik-fail-flg cut-in-flg)
      (setq cut-in-flg nil ik-fail-flg nil interfere-flg nil)
      (warning-message 3 "loop num ~a~%" loopnum)

      ;; 地面食い込み除去
      (setq return-list (avoid-z-cut-in tmp-list free-limb))
      (setq tmp-list (car return-list)  cut-in-flg (cadr return-list) cut-in-list (caddr return-list) )

      ;; 遊脚ik-fail除去
      (setq return-list (detect-non-ik-fail-free-leg tmp-list free-limb :animate animate))
      (setq tmp-list (car return-list) ik-fail-flg (cadr return-list) ik-fail-list (caddr return-list) )

      ;; 自己干渉除去
      (setq return-list (detect-non-interfere-free-leg tmp-list free-limb :animate animate))
      (setq tmp-list (car return-list) interfere-flg (cadr return-list) interfere-list (caddr return-list) )
      (setq interfere-list (make-list (length frame-list) :initial-element nil))

      ;; 平滑化（更新があったところのみ）
      ;; (setq free-leg-coord-list-tmp (smooth-free-leg free-leg-coord-list-tmp ik-fail-list interfere-list))

      (if (select-stream (list *standard-input*) 1.0e-8) (throw :filter nil))
      (incf loopnum)
      );; end while
      
    tmp-list
    ))

;; 地面食い込み (水平化)
(defun avoid-orient-cut-in (frame-list limb)
  (let ( tmp-list term z rpy (z-thresh 100.0) (term-time 0.3) )
    (setq tmp-list (copy-object frame-list))

    (warn "avoid orient cut in")
    (dotimes (piv (length frame-list))
      ;; ヒール・トーポイント以外 漸近水平化
      (when (and (not (= (send (elt frame-list piv) :state limb) *ground*)) (not (= (send (elt frame-list piv) :state limb) *point*)) ) ;; 接地期とポイント期以外
        (setq idx-diff 1 adjacence 0  term (round (/ term-time *ft*)) );; 1 <= adjacence <= term-time
        (while (and (< idx-diff term-time)
                    (<= 0 (- piv idx-diff) ) (<= (+ piv idx-diff) (1- (length frame-list)))
                    (or (not (= (send (elt frame-list (- piv idx-diff)) :state limb) *point*)) (not (= (send (elt frame-list (+ piv idx-diff)) :state limb) *point*))) )
          (setq adjacence (min (- (send (elt frame-list (+ piv idx-diff)) :time) (send (elt frame-list piv) :time))
                               (- (send (elt frame-list piv) :time) (send (elt frame-list (- piv idx-diff)) :time))))
          (incf idx-diff)
          )

        (setq z (send (elt frame-list piv) :z limb) )
        ;; (when (and (< z z-thresh)
        ;;            (or (and (<= 0 (- piv adjacence) ) (= (elt z-list (- piv adjacence)) 50) )
        ;;                (and (<= (+ piv adjacence) (1- (length coord-list))) (= (elt z-list (+ piv adjacence)) 50) ) ) )
        (when (< z z-thresh)
          ;; (warn " ~a" piv)

          (setq adjacence (max (- term-time adjacence) 0));; 0 <= 隣接度 < term-time 近いほうが隣接度大

          (setq rpy (car (send (send (elt frame-list piv) :coord limb) :rpy-angle)))
          (setf (elt rpy 1) (* (elt rpy 1) (- 1 (* (- 1 (/ z z-thresh)) (* 0.5 (+ (cos (* (/ pi term-time) adjacence)) 1))) ) ) );; pitch
          (setf (elt rpy 2) (* (elt rpy 2) (- 1 (* (- 1 (/ z z-thresh)) (* 0.5 (+ (cos (* (/ pi term-time) adjacence)) 1))) ) ) );; roll
          (send (elt tmp-list piv) :coord limb (make-coords :pos (send (elt frame-list piv) :pos limb) :rpy rpy) )
          )

        )
      )(warn "~%")

       tmp-list
    ))

;; 足 地面食い込み除去 接地前後以外にだけ処理を適用する必要がある
(defun avoid-z-cut-in (frame-list limb)
  (let ( tmp-list (z-thresh 100.0) z rpy foot-link footprint-offset  (ankle-z-offset 70)  (cut-in-flg nil ) (collision-distance-thresh 10)
                  (cut-in-list (make-list (length frame-list) :initial-element nil) ) )
    (setq tmp-list (copy-object frame-list))

    (cond
     ( (equal limb :lleg) (setq foot-link *lfoot*  footprint-offset *lfootprint-offset*) )
     ( (equal limb :rleg) (setq foot-link *rfoot*  footprint-offset *rfootprint-offset*) ) )

    (warn "avoid z cut in")

    (dotimes (piv (length frame-list))
      (when (and (not (= (send (elt frame-list piv) :state limb) *ground*)) (not (= (send (elt frame-list piv) :state limb) *non-inpact*)) ) ;; 接地期と衝撃緩和期以外

        ;; 非接地の時、食い込みが無くなるまで高さを上げる
        (send foot-link :move-to (send (elt frame-list piv) :coord limb) :world)
        (send foot-link :locate footprint-offset)
        (while (<= (car (pqp-collision-distance *box* foot-link)) collision-distance-thresh)
          (send foot-link :translate #f(0 0 1) :world);; 脚を上げる
          (send (send (elt tmp-list piv) :coord limb) :translate #f(0 0 1) :world)
          (setq cut-in-flg t)
          )
        (when cut-in-flg (warn " ~a" piv) (setf (elt cut-in-list piv) t))
        )

      );; end dotimes
    (warn "~%")

    (list tmp-list cut-in-flg cut-in-list)
    ))

;; 衝撃緩衝区間についての処理はキーポーズモードでは要変更
;; 遊脚non ik fail軌道生成  footprint ローカル座標でz方向のみ許容
(defun detect-non-ik-fail-free-leg (frame-list limb &key (animate nil) )
  (let (tmp-list ret (ik-fail-flg nil)
                 (ik-fail-list (make-list (length frame-list) :initial-element nil)) )
    (setq tmp-list (copy-object frame-list))
    (warn "detect non ik fail free leg~%")

    (dotimes (piv (length frame-list))
      (when (not (= (send (elt frame-list piv) :state limb) *ground*));; 非接地期のみ ？？？slideとかは？？？
        (send *robot* :move-to (send (elt frame-list piv) :coord :waist) :world);; 腰移動

        ;; ik fail除去
        (cond
         ( (= (send (elt frame-list piv) :state limb) *non-inpact*);; 衝撃緩衝区間
           (setq ret (foot-ik limb 0 piv (send (elt frame-list piv) :coord limb) (list (list t t 2 " pos+rot" )(list t t 3 " mediate" 10)(list :z :z 3 " xy+rp")))) )
         ( t (setq ret (foot-ik limb 0 piv (send (elt frame-list piv) :coord limb) (list (list t t 2 " pos+rot" )(list :z t 3 " xy+rot")(list :z :z 3 " xy+rp")))) ) )
        (when (and (>= ret 1) (<= ret 3));; 1,2,3段階目だけ失敗した時だけ代入
          (send (elt tmp-list piv) :coord limb (send *robot* limb  :end-coords :copy-worldcoords))
          (setf (elt ik-fail-list piv) t)
          (setq ik-fail-flg t)
          )

        );; end when 非接地期
      ;; (when animate (send *pickview* :draw-objects)(unix:usleep (round (* 1000 *ft*)))(x::window-main-one))
      )
    (warn "~%")

    (list tmp-list ik-fail-flg ik-fail-list)
    ))

;; 遊脚ik-fail除去
(defun remove-short-ik-fail-seq (coord-list foot-coord-list z-list lr)
  (let (piv-av end-av end-idx idx next term foot-coord-list-tmp (thre 5) (ik-fail-flg t) (loopnum 0))
    (setq foot-coord-list-tmp (copy-object foot-coord-list))
    (warn "~a~%" lr)

    (while ik-fail-flg
      (warning-message 3 "~%loop num ~a~%" loopnum)(incf loopnum)
      (setq ik-fail-flg nil)
      (dotimes (piv (length coord-list))
        (when (not (= (elt z-list piv) 0));; 非接地時のみ

          ;; piv状態更新
          (send *robot* :move-to (elt coord-list piv) :world);; 腰移動
          ;; (send *robot* :inverse-kinematics (elt foot-coord-list-tmp piv);; ik
          ;;       :move-target (send *robot* lr :end-coords) :link-list (send *robot* :link-list (send *robot* lr :end-coords :parent))
          ;;       :translation-axis t :rotation-axis t :thre thre :warnp nil :debug-view nil)
          (foot-ik lr 0 piv (elt foot-coord-list-tmp piv) (list (list t t 2 "remove shot ik fail not enough")) :disp 0)

          (setq piv-av (send *robot* lr :angle-vector);; av記録
                next (min (1+ piv) (1- (length coord-list))))

          ;; ik-fail区間 end-idx移動
          (send *robot* :move-to (elt coord-list next) :world);; 腰移動
          (setq end-idx (eval next))
          (while (and (< end-idx (length coord-list) )
                      ;; (not (send *robot* :inverse-kinematics (elt foot-coord-list-tmp end-idx)
                      ;;            :move-target (send *robot* lr :end-coords) :link-list (send *robot* :link-list (send *robot* lr :end-coords :parent))
                      ;;            :translation-axis t :rotation-axis t :thre thre :warnp nil :debug-view nil))
                      (> (foot-ik lr 0 end-idx (elt foot-coord-list-tmp end-idx) (list (list t t 2 "")) :disp -1) 0)
                      )
            (setq ik-fail-flg t)
            (incf end-idx)
            (send *robot* :move-to (elt coord-list (min end-idx (1- (length coord-list)))) :world)
            )

          (when (not (= end-idx next));; ik-failがあるとき
            (setq end-av (send *robot* lr :angle-vector));; 終端状態
            (warn "piv ~a  end ~a  " piv end-idx)
            ;; av計算、足先座標代入
            (setq idx (eval next) term (- end-idx piv) )
            (while (< (* (- idx piv) (- idx end-idx) ) 0)
              (warn " ~a" idx)
              (send *robot* :move-to (elt coord-list idx) :world);; 腰移動
              (send *robot* lr :angle-vector (v+ (scale (/ (- idx piv) (float term))  (v- end-av piv-av)) piv-av ));; 関節角度代入
              ;; (send *robot* lr :angle-vector piv-av)
              (setf (elt foot-coord-list-tmp idx) (send *robot* lr :end-coords :copy-worldcoords))
              (incf idx) )
            (setq piv end-idx)
            (warn " remove ik-fail~%")

            )

          );; end 非接地時when
        );; end dotimes
      (if (select-stream (list *standard-input*) 1.0e-8) (throw :filter nil))
      );; end while

    foot-coord-list-tmp
    ))


(defun leg-collision-distance (free-leg-links touch-leg-links)
  (let ( (min-distance most-positive-fixnum) distance )
    (setq idx (1- (length (send *robot* :lleg :links))))

    (block :collision
      (if (< idx 2) (return-from :collision nil))
      (setq distance (car (pqp-collision-distance (car (last free-leg-links)) (elt touch-leg-links 5))))
      (when (< distance min-distance) (setq min-distance distance) (return-from :collision nil))
      (decf idx)
      )

    min-distance
    ))

(defun leg-collision-idx (free-leg-links touch-leg-links)
  (let (idx)
    (setq idx (1- (length (send *robot* :lleg :links))))

    (block :collision
      (when (< idx 2) (setq idx nil)(return-from :collision nil))
      (if (= (rapid-collision-check (car (last free-leg-links)) (elt touch-leg-links idx)) 1) (return-from :collision nil))
      (decf idx)
      )

    idx
    ))

(defun draw-nearest-point ()
  (setq *pqp* (pqp-collision-distance (elt (send *robot* :lleg :links)5) (elt (send *robot* :rleg :links) 5)))
  ;; (setq *cylinder* (make-cylinder 10 (elt *pqp* 0)))
  ;; (send *cylinder* :set-color :yellow)
  ;; (objects (list *robot* (send *m* :model) *lfoot* *rfoot* *box* *cylinder*))
  ;; (send *cylinder* :locate (elt *pqp* 1) :world)
  ;; (setq *axis* (normalize-vector (v* #f(0 0 1) (v- (elt *pqp* 2) (elt *pqp* 1)) )))
  ;; (setq *theta* (acos (v. #f(0 0 1) (v- (elt *pqp* 2) (elt *pqp* 1)) )))
  ;; (send *cylinder* :rotate  *theta* *axis* :world)
  (send *a* :locate (elt *pqp* 1) :world)  (send *b* :locate (elt *pqp* 2) :world)

  )

;; 遊脚期 足干渉回避 ikに失敗しない前提
(defun detect-non-interfere-free-leg ( frame-list free-limb &key (animate nil) )
  (let ( (interfere-flg nil) (piv 0) local-interfere-flg free-leg-links touch-leg-links (interfere-list (make-list (length frame-list) :initial-element nil))
         move-targets check-links (collision-distance-thresh 50) prev-idx target-pos-diff tmp-list
         last-vec-list object-list
         interfere-link-idx vecs vec touch-limb (step-size 5) )

    (warn "interfere " )
    (cond ( (equal free-limb :lleg) (setq touch-limb :rleg ) )
          ( (equal free-limb :rleg) (setq touch-limb :lleg ) ) )
    (setq free-leg-links (send *robot* free-limb :links)  touch-leg-links (send *robot* touch-limb :links))
    (setq tmp-list (copy-object frame-list))
    (setq move-targets (subseq (send *robot* free-limb :links) 5)  check-links (send *robot* touch-limb :links))

    ;; (setq object-list (objects))
    ;; (setq object-list (list (send *m* :model) *robot* *box* *lfoot* *rfoot*))

    (setq *a* (make-cube 10 10 10) *b* (make-cube 10 10 10))
    (send *a* :set-color :red) (send *b* :set-color :green)
    ;; (objects (list (send *m* :model) *robot* *box* *lfoot* *rfoot* *a* *b*))

    (setq last-vec-list (mapcar #'(lambda (val) (setq vecs (pqp-collision-distance (car (last free-leg-links)) val))
                                    (normalize-vector (v- (elt vecs 1) (elt vecs 2))))
                                touch-leg-links) )

    (catch :test
      (dotimes (piv (length frame-list))
        (setq local-interfere-flg nil  prev-idx (max (1- piv) 0) )

        (send *robot* :move-to (send (elt frame-list piv) :coord :waist) :world);; 腰移動
        (foot-ik touch-limb 0 piv (send (elt frame-list piv) :coord touch-limb) (list (list t t 2 " t not")) :disp -1)
        (foot-ik free-limb 0 piv (send (elt frame-list piv) :coord free-limb) (list (list t t 3 " free ik not enough")) :disp 0)

        ;; 差分による目標位置更新
        (setq target-pos-diff (v- (send (elt tmp-list prev-idx) :pos free-limb) (send (elt frame-list prev-idx) :pos free-limb)) )

        (when (not (= (send (elt frame-list piv) :state free-limb) *ground*));; 遊脚期
          ;; 干渉回避
          (while (< (leg-collision-distance free-leg-links touch-leg-links) collision-distance-thresh)
            (setq interfere-flg t  local-interfere-flg t)

            (setq interfere-link-idx (leg-collision-idx free-leg-links touch-leg-links));; 干渉リンク調査

            ;; (if (< (norm target-pos-diff) 5);; 目標足先修正
            ;;     (send (send (elt tmp-list piv) :coord free-limb) :translate (scale step-size (elt last-vec-list interfere-link-idx)) :world)
            ;;   (send (send (elt tmp-list piv) :coord free-limb) :translate (scale step-size (normalize-vector target-pos-diff)) :world) )
            (send (send (elt tmp-list piv) :coord free-limb) :translate (scale step-size (normalize-vector (v- (send (elt tmp-list piv) :pos free-limb) (send (elt tmp-list piv) :pos touch-limb)) )) :world);; keypose
            (foot-ik free-limb 0 piv (send (elt tmp-list piv) :coord free-limb) (list (list t t 2 "  pos+rot")(list :xy t 3 " z+rot")(list :xy nil 3 " z")) :disp 0);; ik実行

            (send (elt tmp-list piv) :coord free-limb (send (send *robot* free-limb :end-coords) :copy-worldcoords))

            (when animate (draw-nearest-point)(send *pickview* :draw-objects)(x::window-main-one)(unix::usleep (* 1000 30)))
            (if (select-stream (list *standard-input*) 1.0e-8) (throw :test nil))
            );; end while


          (when local-interfere-flg (warn " ~a" piv) (setf (elt interfere-list piv) t))
          );; end when

        ;; (setq last-vec-list (mapcar #'(lambda (val) (setq vecs (pqp-collision-distance (car (last free-leg-links)) val))
        ;;                                 (concatenate float-vector (subseq (normalize-vector (v- (elt vecs 1) (elt vecs 2))) 0 2) #f(0) ))
        ;;                             touch-leg-links) )
        (setq last-vec-list (mapcar #'(lambda (val) (setq vecs (pqp-collision-distance (car (last free-leg-links)) val))
                                        (normalize-vector (v- (elt vecs 1) (elt vecs 2))))
                                    touch-leg-links) )

        (when animate (draw-nearest-point)(send *pickview* :draw-objects)(x::window-main-one)(unix::usleep (* 1000 30)))
        (if (select-stream (list *standard-input*) 1.0e-8) (throw :test nil))

        );; end dotimes
      )

    ;; (objects object-list)
    (if (not interfere-flg) (warn " nan"))
    (warn "~%")

    (list tmp-list interfere-flg interfere-list)
    ))

;; 遊脚期用平滑化
(defun smooth-free-leg ( coord-list  &rest lists )
  (let* ( coord-list-tmp (mod-list '(1 1 1 1 1)) diff sum  )
    (setq coord-list-tmp (copy-object coord-list))
    (warn "smooth ")
    (dotimes (piv (length coord-list))
      (when (some #'eval (mapcar #'(lambda (val) (elt val piv)) lists) )
        (setq sum (make-coords)  diff (/ (length mod-list) 2) )
        (warn "~a (" piv)
        (dolist (mod mod-list);; 平滑化処理
          (if (and (> (+ piv diff) -1) (< (+ piv diff) (length coord-list)))
              (setq sum (funcall 'coord+ sum (funcall 'scale-coord mod (elt coord-list (+ piv diff)))))
            (setq sum (funcall 'coord+ sum (elt coord-list piv))))
          (warn " ~a" (+ piv diff))
          (decf diff))
        (warn ") ")
        (setf (elt coord-list-tmp piv) (funcall 'scale-coord (/ 1.0 (length mod-list)) sum))
        );; end when
      );; end dotimes
    (warn "~%")

    coord-list-tmp
    ))



;; 衝撃緩和軌道 ik-fail 除去;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun decide-non-inpact ( coord-list lcoord-list rcoord-list lz-list rz-list )
  (let ( coord-list-tmp )
    (setq coord-list-tmp (copy-object coord-list))
    (warn "waist ")
    (dotimes (piv (length coord-list))
      (when (or (= (elt lz-list piv) *non-inpact*) (= (elt rz-list piv) *non-inpact*) )
        (send *robot* :move-to (elt coord-list piv) :world)
        (waist-ik 0 piv (elt coord-list piv) (elt lcoord-list piv) (elt rcoord-list piv) (list (list t t 3 "" 2)))
        (setf (elt coord-list-tmp piv) (send (send *robot* :worldcoords) :copy-worldcoords) )
        );; end when
      );; end dotimes
    (warn "~%")

    coord-list-tmp
    ))

;; 今一
(defun waist-ik (depth piv coord lcoord rcoord val-list &key (state -1) (disp 0) );; disp 0:全表示 1:接地脚のみ表示 -1:非表示
  (let ( (ik-fail-sign 0) (thre 5) l-end-link r-end-link l-links r-links waist-link-list rleg-link-list );; ik-fail-sign 失敗段階数 0ならik成功
    (when (= (length (elt val-list depth)) 5) (setq thre (elt (elt val-list depth) 4)));; 指定があるときのみ精度代入

    ;; end link
    (setq l-end-link (send *robot* :lleg :end-coords :parent) r-end-link (send *robot* :rleg :end-coords :parent))
    (when (= (length (send *robot* :lleg :links)) 7) (setq l-end-link (send l-end-link :parent) r-end-link (send r-end-link :parent)))

    (setq l-links (send *robot* :link-list l-end-link) r-links (send *robot* :link-list r-end-link) )
    ;; (setq waist-link-list (append (cdr (reverse l-links)) (list (send (car l-links) :parent)) ) rleg-link-list (append waist-link-list r-links) )

    ;; (send *robot* :fix-leg-to-coords :lleg lcoord);; 左足がroot

    (when (not (send *robot* :fullbody-inverse-kinematics (list lcoord rcoord )
                     :move-target (list (send *robot* :lleg :end-coords) (send *robot* :rleg :end-coords) )
                     :link-list (list l-links r-links)
                     :translation-axis (list t t) :rotation-axis (list t t)
                     :thre (list thre thre) :warnp nil :debug-view nil ) )
      (incf ik-fail-sign)
      (when (or (and  (= disp 1) (or (= state 0) (and (= state 1) (equal lr :lleg)) (and (= state 2) (equal lr :rleg)) ) );; 接地脚のみ表示
                (= disp 0) );; 全表示
          (warning-message (elt (elt val-list depth) 2) (elt (elt val-list depth) 3)) (warn " ~a" piv))
      (if (< (1+ depth) (length val-list)) (setq ik-fail-sign (+ ik-fail-sign (foot-ik-no-collision-check lr (1+ depth) piv coord val-list :state state :disp disp)))))
    ;; 一つ条件の緩いikを実行
    ik-fail-sign
    ))


;; 歩幅調整
(defun horizontal-scale (frame-list scale)
  (let ( pos (limb-list '(:waist :lleg :rleg)) )
    (dolist (limb limb-list)
      (dotimes (piv (length frame-list))
        (setq pos (send (elt frame-list piv) :pos limb))
        (send (elt frame-list piv) :pos limb (float-vector (* (elt pos 0) scale) (* (elt pos 1) scale) (elt pos 2)))
        )
      )
    frame-list
    ))

;; 骨格情報からzmp計算
(setq *total-mass* 60)
(defun calc-zmp (frame-list &key (old nil));; [mm]
;; (defun calc-zmp-from-skeleton-animation (animation &key (old nil));; [mm]
  (let ( (pose-list nil) link-mass-list  link-pos-list joint-pos-list av (zmp-list nil) (g (elt *g-vec* 2))
         cur-idx prev-idx next-idx  link-pos prev-link-pos next-link-pos link-acc  x-nume y-nume denom link-flg-list)

    ;; 頭部7％、胴体43％、上肢6.5％（上腕3.5％、前腕2.3％、掌・手指0.8％）、下肢18.5％（大腿11.6％、下腿5.3％、1.8％）
    (cond
     (old (setq link-mass-list (list 0.43  0.116 0.053 0.018  0.116 0.053 0.018  0.035 0.023  0.035 0.023  0.07))
          (setq link-flg-list  (list nil nil  t t t nil  t t t nil  t t nil    t t nil    nil)) )
     (t   (setq link-mass-list (list 0.43  0.116 0.053 0.018  0.116 0.053 0.018  0.035 0.023 0.008  0.035 0.023 0.008  0.07))
          (setq link-flg-list  (list nil nil  t t t nil  t t t nil  t t t nil  t t t nil  nil)) ) )
    (setq link-mass-list (mapcar #'(lambda (val) (* val *total-mass*)) link-mass-list))

    ;; pose-list作成
    (dotimes (piv (length frame-list))
      ;; (setq av (elt animation piv)  link-pos-list nil  joint-pos-list nil)
      (setq link-pos-list nil  joint-pos-list (send (elt frame-list piv) :joint-pos-list))

      ;; link-pos-list作成
      ;; (push-back (subseq av 0 3) link-pos-list);; 腰
      (push-back (elt joint-pos-list 0) link-pos-list);; 腰
      ;; (dotimes (i (/ (length av) 3)) (push-back (coerce (subseq av (* i 3) (* (1+ i) 3)) float-vector) joint-pos-list) );; joint-pos-list
      (dotimes (i (1- (length joint-pos-list)))
      ;; (dotimes (i (length joint-pos-list))
        (when (elt link-flg-list i)
          (push-back (scale 0.5 (v+ (elt joint-pos-list (1+ i)) (elt joint-pos-list i))) link-pos-list) ) );; 各肢体

      (push-back link-pos-list pose-list)
      )

    ;; zmp計算
    (dotimes (cur-idx (length pose-list))
      (setq x-nume 0 y-nume 0 denom 0)
      (setq prev-idx (max 0 (1- cur-idx))  next-idx (min (1- (length pose-list)) (1+ cur-idx)))

      (dotimes (i (length link-pos-list));; 各リンク走査
      ;; (dotimes (i 1);; 各リンク走査
        (setq link-pos      (elt (elt pose-list cur-idx ) i)
              prev-link-pos (elt (elt pose-list prev-idx) i)
              next-link-pos (elt (elt pose-list next-idx) i)
              link-acc (scale (/ 1 (expt *ft* 2)) (v- (v+ next-link-pos prev-link-pos) (scale 2 link-pos))) );; [mm/sec^2]

        (setq x-nume (+ x-nume  (* (elt link-mass-list i) (- (* (+ (elt link-acc 2) g) (elt link-pos 0))  (* (elt link-pos 2) (elt link-acc 0)) )) )
              y-nume (+ y-nume  (* (elt link-mass-list i) (- (* (+ (elt link-acc 2) g) (elt link-pos 1))  (* (elt link-pos 2) (elt link-acc 1)) )) )
              denom (+ denom (* (elt link-mass-list i) (+ (elt link-acc 2) g) ) )
              )

        );; end リンク走査

      (push-back (float-vector (/ x-nume denom) (/ y-nume denom) 0) zmp-list)
      ;; (setq zmp-x (eval (/ x-nume denom)) zmp-y (eval (/ y-nume denom)))
      (send (elt frame-list cur-idx) :zmp (float-vector (/ x-nume denom) (/ y-nume denom) 0))
      )
    ;; zmp-list
    frame-list
  ))

;; キーポーズ抽出
(defun extract-keyframe ( frame-list )
  (let ( (keyframe-list nil) prev-state keypose-flg (mod-list '(1 1 1)) forward-avg back-avg add-idx )
    (push-back (copy-object (elt frame-list 0)) keyframe-list);; 初期状態
    (dotimes (piv (length frame-list))
      (setq keypose-flg nil add-idx (eval piv) )

      ;; 速度極小から抽出
      ;; (dolist (limb '(:waist :lleg :rleg))
      (dolist (limb '(:lleg :rleg :larm :rarm))
      ;; (dolist (limb '(:larm))
        (setq forward-avg 0 back-avg 0);; 前後近傍の速度平均
        (dotimes (diff (length mod-list))
          (setq forward-avg (+ forward-avg (send (elt frame-list (min (+ piv (1+ diff)) (1- (length frame-list)))) :vel limb))
                back-avg    (+ back-avg    (send (elt frame-list (max (- piv (1+ diff)) 0)) :vel limb)) )
          )
        (setq forward-avg (/ forward-avg (length mod-list)) back-avg (/ back-avg (length mod-list)))

        (when (and (not (= (send (elt frame-list piv) :state limb) *ground*)) (not (= (send (elt frame-list piv) :state limb) *touch*)) (not (= (send (elt frame-list piv) :state limb) *point*)) ;; 非接地期
                 ;; (< (send (elt frame-list piv) :vel limb) forward-avg) (< (send (elt frame-list piv) :vel limb) back-avg);; 前後近傍
                 (< (send (elt frame-list piv) :vel limb) (send (elt frame-list (min (1+ piv) (1- (length frame-list)))) :vel limb));; 直後
                 (< (send (elt frame-list piv) :vel limb) (send (elt frame-list (max (1- piv) 0)) :vel limb)) );; 直前
          (send (elt frame-list piv) :push-keypose-flg limb)
          (setq keypose-flg t))
        )

      ;; 接地状態遷移から抽出
      (dolist (limb '(:lleg :rleg))
        ;; (when (not (= (send (elt frame-list (max (1- piv) 0)) :state limb) (send (elt frame-list piv) :state limb)))
        ;;   (when (< (send (elt frame-list (max (1- piv) 0)) :state limb) (send (elt frame-list piv) :state limb))
        ;;     (setq add-idx (max (1- piv) 0)) );; 静止状態をキーポーズとして優先
        ;;   (send (elt frame-list add-idx) :push-keypose-flg limb);; 不要？？
        ;;   (setq keypose-flg t))
        (cond
         ( (and (not (= (send (elt frame-list (max (1- piv) 0)) :state limb) (send (elt frame-list piv) :state limb)))
                (> (send (elt frame-list (max (1- piv) 0)) :state limb) (send (elt frame-list piv) :state limb)) )
           (send (elt frame-list add-idx) :push-keypose-flg limb)
           (setq keypose-flg t) )
         ( (and (not (= (send (elt frame-list (min (1+ piv) (1- (length frame-list)))) :state limb) (send (elt frame-list piv) :state limb)))
                (> (send (elt frame-list (min (1+ piv) (1- (length frame-list)))) :state limb) (send (elt frame-list piv) :state limb)) )
           (send (elt frame-list add-idx) :push-keypose-flg limb)
           (setq keypose-flg t) )
         )
        )

      (if keypose-flg (push-back (copy-object (elt frame-list add-idx)) keyframe-list))
      )
    (push-back (copy-object (car (last frame-list))) keyframe-list);; 終端状態
    
    keyframe-list
    ))
    


;; リストをgnuplot用に保存
(defun save-list  (&rest lists)
  (let ( plotpath gfile gf (height 0.5) span)

    (setq span *ft*)
    ;; (setq span 1)
    (setq plotpath (concatenate string "/home/" (unix::getenv "USER") "/Dropbox/choreonoid/" (send *robot* :name) "/" *motionname* "/")
          gfile (concatenate string plotpath *motionname* ".g") )

    (if (not (probe-file plotpath)) (unix::system (concatenate string "mkdir " plotpath )));; ディレクトリがない場合
    (if (not (probe-file gfile))
        (with-open-file
         (gf gfile :direction :output :if-does-not-exist :create)
         (format gf "set term postscript color~%")
         (format gf "set output \"hoge.eps\"~%")
         (format gf "set grid~%")
         (format gf "set size 1,~a~%" (* height (length lists)))
         (format gf "set multiplot layout ~a,1~%" (length lists))
         (dotimes (i (length lists))
           (format gf "~%set origin 0,~a;set size 1,0.5;set xlabel \"time[sec]\"; set ylabel \"[mm]\";set title \"hoge\";~%plot \"~a.dat\" using 1:~a w p notitle;~%" 
                   (* height (- (length lists) i 1)) *motionname* (+ 2 i))
           )
         (format gf "~%unset multiplot~%")
         (format gf "~%~%set output~%")
         (format gf "~%set term x11~%")
         (format gf "!gnome-open *.eps ~%!cp *.eps \~/Pictures/thesis/graph/~%" )
         )
      )

    (with-open-file
     (datf (concatenate string plotpath *motionname* ".dat") :direction :output :if-does-not-exist :create)

     (dotimes (piv (length (car lists)))
       (format datf "~a " (* piv span))
       (dolist (target lists)
         (format datf " ~a" (elt target piv))
         )
       (format datf "~%")
       )
     )
    ))

;; (setq rel-leg-x-list nil)(mapc #'(lambda (pos leg) (push-back (- (elt leg 0) (elt pos 0)) rel-leg-x-list)) lpos-list rpos-list)
;; (setq rel-zmp-x-list nil)(mapc #'(lambda (pos zmp) (push-back (- (elt zmp 0) (elt pos 0)) rel-zmp-x-list)) lpos-list zmp-list)

;; frameのリストのメンバ変数をコピー
(defun copy-frame-slot (frame-list-from frame-list-to slots limbs)
  (let ()
    (dotimes (i (length frame-list-from))
      ;; (send (elt rframe-list i) :state :lleg (send (elt mframe-list i) :state :lleg))
      ;; (send (elt rframe-list i) :state :rleg (send (elt mframe-list i) :state :rleg))
      ;; (send (elt rframe-list i) :vel :lleg (send (elt mframe-list i) :state :lleg))
      ;; (send (elt rframe-list i) :vel :rleg (send (elt mframe-list i) :state :rleg))
      (dolist (limb limbs)
        (dolist (slot slots)
                (send (elt frame-list-to i) slot limb (send (elt frame-list-from i) slot limb))
                ))
      )
    frame-list-to
    ))