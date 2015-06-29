;; bvhからpseq(choreonoid用)を生成する
#!/usr/bin/env/ roseus
#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)
(ros::roseus "bvh2pseq")

(load "euslib/demo/k-kojima/bvh/str-for-pseq.l")
(load "euslib/demo/k-kojima/bvh/filter.l")
(load "euslib/demo/k-kojima/bvh/perfume-bvh-robot-model.l")
(load "euslib/demo/k-kojima/bvh/kinect-sdk-sub-model-viewer.l")
(load "euslib/demo/k-kojima/bvh/frame-class.l")
(load "euslib/demo/k-kojima/dynamic_control/log-tools.l")
(load "package://roseus_tutorials/src/kinect-model-viewer.l")

(load "euslib/jsk/gnuplotlib.l")

;; save choreonoid project file at "$HOME / $CHOREONOID_WORKSPACE / (robot's name) / (motion-name) /"
;; save bvhfile at "$HOME / $BVH_PATH"

;; (if (not (boundp '*a*)) (setq *a* (make-cube 200 400 1)))

(when (not (boundp '*gnuplot*)) (setq *gnuplot* (gnuplot))(send *gnuplot* :command "set size 1,1")(send *gnuplot* :command "set grid"))

;; (load  "package://jsk_hrpsys_ros_bridge/scripts/hrp4r-interface.l")
;; (if (not (boundp '*hrp4r*)) (setq *robot* (hrp4r)))

;; (if (not (boundp '*hrp2*))
;;     (setq *robot* (hrp2jsk-simple-detail)))

(when (not (boundp '*hrp2*)) (setup-model-for-hrp2) (setq *robot* *hrp2*))

(send *robot* :make-detail-collision-model)
(send *robot* :make-detail-collision-model :collision-func 'pqp-collision-check)

;; (require :p1 "package://jsk_hrpsys_ros_bridge/models/p1.l")
;; (if (not (boundp '*p1*)) (setq *robot* (p1)))

(when (not (boundp '*box*));; 床
    (setq *box* (make-cube 2000 1500 1))
    (send *box* :locate #f(1750 -250 -1.6))
    (send *box* :set-color :gray))

(when (not (boundp '*lfoot*));;左足目標位置
    (setq *lfoot* (make-cube 260 160 4))
    (send *lfoot* :set-color :red))
(when (not (boundp '*rfoot*));;右足目標位置
    (setq *rfoot* (make-cube 260 160 4))
    (send *rfoot* :set-color :green))

(when (not (boundp '*lfootprint-offset*))
  (setq *lfootprint-offset* #f(10 10 2))
  (setq *rfootprint-offset* #f(10 -10 2)))

;;bvh膝
(when (not (boundp '*knee*)) (setq *knee* (make-cube 50 50 50)) (send *knee* :set-color :pink))
;;bvh左足首
(when (not (boundp '*ankle*)) (setq *ankle* (make-cube 50 50 50)) (send *ankle* :set-color :pink))
;;bvh左爪先
(when (not (boundp '*toe*)) (setq *toe* (make-cube 50 50 50)) (send *toe* :set-color :pink))

(when (not (boundp '*pickview*)) (pickview)(send *pickview* :resize 800 600)(send *pickview* :whiteback t))

(setq stdout nil)
;; (setq *touch-flg* nil)
(setq buf "")
(if (not (boundp '*start-frame*)) (setq *start-frame* 0));; 処理開始frame番号

(defun coord+ (coord1 coord2)
  (make-coords
   :pos (v+ (send coord1 :worldpos) (send coord2 :worldpos))
   :rot (m+ (send coord1 :worldrot) (send coord2 :worldrot))))
(defun scale-coord (k coord)
  (make-coords
   :pos (scale k (send coord :worldpos))
   :rot (scale-matrix k (send coord :worldrot))))

;; pseqファイルへのバッファ生成
(defun write-buf ( dt &key (speed 1) (init nil) (keypose-flg-list '(:lleg :rleg :larm :rarm)) )
  (let ( av joints-list-total joints-list joints buf q-list-total q-list q-str)

    (setq av nil)    
    (cond;; qリスト(全体)作成
     ((equal (send *robot* :name) "HRP2JSKNT")
      (setq av (send *robot* :hrp-angle-vector))
      (setq q-list-total (mapcar #'deg2rad (coerce (concatenate float-vector (subseq av 0 18) #f(0 -10 20 -60 0 0 0 0  0 10 -20 -60 0 0 0 0 ) ) cons))))
     ((equal (send *robot* :name) "HRP2JSK") (setq q-list-total (mapcar #'deg2rad (coerce (send *robot* :hrp-angle-vector) cons))))
     ((equal (send *robot* :name) "P1")
      (setq av (send *robot* :angle-vector)) (setq av (concatenate float-vector (subseq av 12) (subseq av 0 12)))
      (setq q-list-total (mapcar #'deg2rad (coerce av cons))))
     (t (setq q-list-total (mapcar #'deg2rad (coerce (send *robot* :angle-vector) cons)))))

    ;; jointsリスト(全体)作成
    (setq joints-list-total nil)
    (dotimes (i (length q-list-total)) (push-back i joints-list-total))

    ;; joints,qの抜粋
    ;; iklinks作成
    (setq ikLinks nil q-list nil joints-list nil)
    (setq iklinks "      ikLinks:~%")
    (cond
     ( (equal (send *robot* :name) "HRP4R");; hrp4
       (setq iklinks (add-iklinks iklinks "WAIST" 0 :torso :end-coords :offset-vec #f(0 0 -210)))
       (when (member :rleg keypose-flg-list)
         (setq iklinks (add-iklinks iklinks "R_FOOT" 7 :rleg :ankle-r :touch-flg (cadr *touch-flg*))) )
       (when (member :lleg keypose-flg-list)
         (setq iklinks (add-iklinks iklinks "L_FOOT" 14 :lleg :ankle-r :touch-flg (car *touch-flg*))) )
       (when (member :rarm keypose-flg-list)
         (setq iklinks (add-iklinks iklinks "R_WRIST_R" 25 :rarm :wrist-r  )) )
       (when (member :larm keypose-flg-list)
         (setq iklinks (add-iklinks iklinks "L_WRIST_R" 42 :larm :wrist-r  )) )
       ;; joints-list, q-listの抜粋 未実装
       )
     ( (equal (send *robot* :name) "HRP2JSK");; hrp2jsk
       (when (or (member :lleg keypose-flg-list) (member :rleg keypose-flg-list));; 腰
         (setq iklinks (add-iklinks iklinks "WAIST" 0 nil nil :offset-vec #f(0 0 0) )) )
       (when (member :rleg keypose-flg-list);; 右足
         (push-back (subseq joints-list-total 0 6) joints-list) (push-back (subseq q-list-total 0 6) q-list)
         (setq iklinks (add-iklinks iklinks "RLEG_JOINT5" 6 :rleg :ankle-r :touch-flg (cadr *touch-flg*))) )
       (when (member :lleg keypose-flg-list);; 左足
         (push-back (subseq joints-list-total 6 12) joints-list) (push-back (subseq q-list-total 6 12) q-list)
         (setq iklinks (add-iklinks iklinks "LLEG_JOINT5" 12 :lleg :ankle-r :touch-flg (car *touch-flg*))) )
       (push-back (subseq joints-list-total 12 16) joints-list) (push-back (subseq q-list-total 12 16) q-list);; :torso :headは今はデフォルト
       (when (member :rarm keypose-flg-list);; 右手
         (push-back (subseq joints-list-total 16 24) joints-list) (push-back (subseq q-list-total 16 24) q-list)
         (setq iklinks (add-iklinks iklinks "RARM_JOINT6" 23 :rarm :wrist-p )) )
       (when (member :larm keypose-flg-list);; 左手
         (push-back (subseq joints-list-total 24 32) joints-list) (push-back (subseq q-list-total 24 32) q-list)
         (setq iklinks (add-iklinks iklinks "LARM_JOINT6" 31 :larm :wrist-p )) )
       )
     ( (equal (send *robot* :name) "HRP2JSKNT");; hrp2jsknt
       (setq iklinks (add-iklinks iklinks "WAIST" 0 nil nil :offset-vec #f(0 0 0) ))
       (setq iklinks (add-iklinks iklinks "RLEG_JOINT5" 6 :rleg :ankle-r :touch-flg (cadr *touch-flg*)))
       (setq iklinks (add-iklinks iklinks "LLEG_JOINT5" 13 :lleg :ankle-r :touch-flg (car *touch-flg*)) )
       (setq iklinks (add-iklinks iklinks "RARM_JOINT6" 25 :rarm :wrist-p ))
       (setq iklinks (add-iklinks iklinks "LARM_JOINT6" 33 :larm :wrist-p ))
       ;; joints-list, q-listの抜粋 未実装
       )
     ( (equal (send *robot* :name) "P1");; p1
       (setq iklinks (add-iklinks iklinks "WAIST" 0 :body nil :offset-vec #f(0 0 0)))
       (setq iklinks (add-iklinks iklinks "RARM_JOINT7" 8 :rarm :wrist-r  ))
       (setq iklinks (add-iklinks iklinks "LARM_JOINT7" 16 :larm :wrist-r  ))
       (setq iklinks (add-iklinks iklinks "RLEG_JOINT5" 22 :rleg :ankle-r :touch-flg (cadr *touch-flg*)))
       (setq iklinks (add-iklinks iklinks "LLEG_JOINT5" 28 :lleg :ankle-r :touch-flg (car *touch-flg*)))
       ;; joints-list, q-listの抜粋 未実装
       )
     )

    (setq q-str (list2str (flatten q-list) "q"));; qリストをストリングに変換
    (setq joints (list2str (flatten joints-list) "joints"));; jointsリストをストリングに変換

    ;; (if (= frame-no 0) (setq *time* 0) (setq *time* (+ 1 (/ (* frame-no *ft*) speed)) ) )
    (if init (setq *time* 0) (setq *time* (+ *time* (/  dt speed)) ) )

    ;;バッファ書き込み
    (setq buf (concatenate string
                           "  -~%"
                           "    time: " (string *time*) "~%"
                           "    refer:~%"
                           "      type: Pose~%"
                           "      name: \"\"~%"
                           "      " joints "~%"
                           "      " q-str "~%"
                           iklinks
                           ))

    buf
    ))

;; モデルのリンク長等を求める
(defun init-model-param (animation)
  (let ( (Lh 0) (Lr 0) (ankle-height 0) (foot-vec #f(0 0 0)) x y x_)

    ;; ロボットを初期位置姿勢へ
    (send *robot* :move-to (make-coords) :world)
    (mapc #'(lambda (joint) (send joint :joint-angle 0)) (send *robot* :lleg :joint-list))

    ;; 足首から足先へのオフセットベクトル
    (setq *foot-offset-vec* (v- (send *robot* :lleg :end-coords :worldpos) (send *robot* :lleg :ankle-r :worldpos)))

    ;; 上腿・下腿長さ
    (setq Lh (apply #'+  (subseq (mapcar #'(lambda (link) (norm (v- (send link :worldpos) (send (send link :parentcoords) :worldpos) )))
                                      (send *m* :model :lleg) ) 1 3)) )
    (setq Lr (norm (v- (send (car (send *robot* :lleg :links)) :worldpos) (send (elt (send *robot* :lleg :links) 5) :worldpos))));; 足首まで

    ;; 股関節から足首までのオフセットベクトル（左足基準）
    (setq *thigh-offset-vec* (reduce #'(lambda (vec link) (v+ vec (v- (send link :worldpos) (send (send link :parentcoords) :worldpos))))
            (append (list #f(0 0 0)) (cdr (send *robot* :lleg :links)))))
    (setf (elt *thigh-offset-vec* 0) 0)(setf (elt *thigh-offset-vec* 2) 0);; y成分のみ

    ;; 腰から股関節までのオフセットベクトル（左足基準）
    (setq *hip-offset-vec* (v- (send (car (send *robot* :lleg :links)) :worldpos) (send *robot* :worldpos)))
    (setq *hip-offset-vec* (send (send *robot* :worldcoords) :inverse-rotate-vector *hip-offset-vec*))


    ;; 初期足首高さ・足面ピッチ角度計算 最初の10frameの平均
    (dotimes (i 10)
      (setq ankle-height (+ ankle-height (elt (elt animation i) 14) (elt (elt animation i) 26) ))

      (setq foot-vec (v+ foot-vec
                         (v- (subseq (elt animation i) 15 18) (subseq (elt animation i) 12 15))
                         (v- (subseq (elt animation i) 27 30) (subseq (elt animation i) 24 27))))
      )
    (setq ankle-height (/ ankle-height 20))
    (setq x (normalize-vector foot-vec))
    (setq y_ (normalize-vector (v* #f(0 0 1) x)))
    (setq x_ (normalize-vector (v* y_ #f(0 0 1))))
    (setq *pitch*  (rad2deg (acos (v. x_ x))))

    (setq *alpha* (/ Lr Lh));; 上腿・下腿長さ比
    (setq *beta* (/ (+ Lr (abs (elt *foot-offset-vec* 2))) (+ Lh ankle-height)));; 腰高さ比


    (setq *wh*
          (v- (scale 0.5 (v+ (send (car (send *m* :model :lleg)) :worldpos) (send (car (send *m* :model :rleg)) :worldpos) ))
              (send (car (send *m* :model :torso)) :worldpos) )
          *wr*
          (v- (scale 0.5 (v+ (send (car (send *robot* :lleg :links)) :worldpos) (send (car (send *robot* :rleg :links)) :worldpos) ))
              (send *robot* :worldpos) ));; hrp2は:worldposでいいが他のロボットは不明
    ))

 
;; 膝関節軸方向をあらわすyベクトル計算 回帰平面利用
(defun calc-y (leg-pos-list coord)
  (let (u l f yul yfu yfl y n)
     ;; 体中心座標系に変更
     (dotimes (i (length leg-pos-list)) (setf (elt leg-pos-list i) (send coord :inverse-rotate-vector (elt leg-pos-list i))))

     (setq u (normalize-vector (v- (elt leg-pos-list 1) (elt leg-pos-list 0)))
           l (normalize-vector (v- (elt leg-pos-list 2) (elt leg-pos-list 1)))
           f (normalize-vector (v- (elt leg-pos-list 3) (elt leg-pos-list 2))) )

    (setq yul (normalize-vector (v* u l))
          yfu (normalize-vector (v* f u));; yfは伸膝時しか使わないからlよりもuの方が精度がいいはず
          yfl (normalize-vector (v* f l)));; 最後の関節は負方向しか回らないので逆順の外積

    (setq yul (scale (/ (v. yfl yul) (abs (v. yfl yul))) yul));; 膝が逆関節の時はyulを逆向きにする

    ;; (setq y yul);; とりあえず
    (setq y yfl)
    ;; (setq y yfu);; これだけにするのは膝を曲げた時に不味い

    ;; (setq n (calc-regression-plane leg-pos-list));; 回帰平面法線ベクトル
    (setq n (car (lmeds leg-pos-list)));; 回帰平面法線ベクトル
    (cond ( (< (v. y n) 0) (setq y (scale -1 n)) ) (t (setq y n)) )

    y
    ))

;; ロボットと棒人間の関節構造の違いを考慮して目標腰・足先座標を作成
(defun copy-leg-state (av  &key (z-offset 0) )
  (let ( pos rot waist-coord hip ankle knee toe foot x y z target-coord feet-coords robot-waist-coord robot-hip thigh-offset-vec )

     (setq waist-coord (send (car (send *m* :model :torso)) :copy-worldcoords))
     (setq robot-waist-coord (make-coords :pos (scale *beta* (send (car (send *m* :model :torso)) :worldpos)) ;; 腰の移動量を脚長さの比に合わせる
                                          :rot (send (car (send *m* :model :torso)) :worldrot)) )
     (send robot-waist-coord :translate (v- *wh* *wr*) :local);; ロボットと棒人間で股関節中心を揃える

     ;; 初期姿勢からデフォルト腰下げ高さを計算 (最初のみ)
     (when (not *waist-z-offset*)
         (send *robot* :lleg :angle-vector (make-array (length (send *robot* :lleg :angle-vector)) :element-type 'float :initial-element 0))
         (send *robot* :fix-leg-to-coords (make-coords) :lleg)
         (setq *waist-z-offset* (- (elt (send *robot* :worldpos) 2) (elt (send robot-waist-coord :worldpos) 2)))
         )
 
     ;; 目標足先座標計算
     (setq feet-coords
           (mapcar #'(lambda (hip knee ankle toe lr)
                       (setq leg-pos-list (list hip knee ankle toe))

                       ;; 足先位置 体中心座標系
                       (setq hip (send waist-coord :inverse-rotate-vector hip)
                             ankle (send waist-coord :inverse-rotate-vector ankle)
                             toe (send waist-coord :inverse-rotate-vector toe))
                       (setq pos (v- ankle hip);; 股関節から足首
                             foot (v- toe ankle))

                       ;; 足先姿勢計算 体中心座標系
                       (setq y (calc-y leg-pos-list waist-coord));; y計算 回帰平面利用
                       (setq z (normalize-vector (v* foot y)));; z計算
                       (setq x (normalize-vector (v* y z)));; x計算
                       (setq rot (transpose (matrix x y z)))

                       ;; 目標座標作成
                       (setq target-coord (make-coords :rot rot))

                       (send target-coord :rotate (deg2rad (* -1 *pitch*) ) :y :local);; 足首のオフセット角度分回転

                       (send target-coord :locate (scale *alpha* pos) :world);; 股関節からの足首の移動分 足長考慮

                       (setq thigh-offset-vec (copy-seq *thigh-offset-vec*));; 股関節から足首のオフセット分（水平方向）平行移動
                       (if (equal lr :rleg) (setf (elt thigh-offset-vec 1) (* -1 (elt thigh-offset-vec 1))))
                       (send target-coord :translate thigh-offset-vec :local)

                       (send target-coord :translate *foot-offset-vec* :local);; 足のオフセット分平行移動

                       (setq robot-hip (copy-seq *hip-offset-vec*));; 腰から股関節の移動分
                       (if (equal lr :rleg) (setf (elt robot-hip 1) (* -1 (elt robot-hip 1))))
                       (send target-coord :translate robot-hip :world)

                       (send target-coord :transform robot-waist-coord :world);; 体中心座標系からワールド座標系に変更

                       target-coord;; 返り値
                       );; end lambda
                   (list (subseq av 6 9) (subseq av 18 21));; hip
                   (list (subseq av 9 12) (subseq av 21 24));; knee
                   (list (subseq av 12 15) (subseq av 24 27));; ankle
                   (list (subseq av 15 18) (subseq av 27 30));; toe
                   (list :lleg :rleg)
                   ))
     (send *lfoot* :move-to (elt feet-coords 0) :world);; 目標足位置表示
     (send *rfoot* :move-to (elt feet-coords 1) :world)


     (if (boundp '*lankle*) (send *lankle* :locate (subseq av 12 15) :world))
     (if (boundp '*ltoe*) (send *ltoe* :locate (subseq av 15 18) :world))

     (send robot-waist-coord :translate (float-vector 0 0 *waist-z-offset*) :world);; デフォルト腰高さ調整
     (send robot-waist-coord :translate (float-vector 0 0 z-offset) :world)

     ;; 腰の姿勢変化はヨー軸周りのみ
     ;; (setq robot-waist-coord (make-coords :pos (send robot-waist-coord :worldpos) :rpy (list (elt (car (send waist-coord :rpy-angle)) 0) 0 0)) )

     ;; (push (elt feet-coords 0) lcoord-list)
     ;; (push (elt feet-coords 1) rcoord-list)
     ;; (push robot-waist-coord coord-list)
     (push robot-waist-coord feet-coords)

     feet-coords
    ))

;; 各種グローバルリスト宣言
(defun declare-lists ()
  (setq neck-p-vel (list 0) neck-y-vel (list 0) waist-p-vel (list 0) waist-y-vel (list 0)
        l-crotch-r-vel (list 0) l-crotch-p-vel (list 0) l-crotch-y-vel (list 0)
        l-knee-p-vel (list 0) l-ankle-r-vel (list 0) l-ankle-p-vel (list 0) l-toe-p-vel (list 0)
        r-crotch-r-vel (list 0) r-crotch-p-vel (list 0) r-crotch-y-vel (list 0)
        r-knee-p-vel (list 0) r-ankle-r-vel (list 0) r-ankle-p-vel (list 0) r-toe-p-vel (list 0)
        l-shoulder-r-vel (list 0) l-shoulder-p-vel (list 0) l-shoulder-y-vel (list 0)
        l-elbow-p-vel (list 0) l-wrist-r-vel (list 0) l-wrist-p-vel (list 0) l-wrist-y-vel (list 0)
        r-shoulder-r-vel (list 0) r-shoulder-p-vel (list 0) r-shoulder-y-vel (list 0)
        r-elbow-p-vel (list 0) r-wrist-r-vel (list 0) r-wrist-p-vel (list 0) r-wrist-y-vel (list 0)

        ;; neck-p-tq (list 0) neck-y-tq (list 0) waist-p-tq (list 0) waist-y-tq (list 0)                               
        ;; l-crotch-r-tq (list 0) l-crotch-p-tq (list 0) l-crotch-y-tq (list 0)                     
        ;; l-knee-p-tq (list 0) l-ankle-r-tq (list 0) l-ankle-p-tq (list 0) l-toe-p-tq (list 0)        
        ;; r-crotch-r-tq (list 0) r-crotch-p-tq (list 0) r-crotch-y-tq (list 0)                     
        ;; r-knee-p-tq (list 0) r-ankle-r-tq (list 0) r-ankle-p-tq (list 0) r-toe-p-tq (list 0)        
        ;; l-shoulder-r-tq (list 0) l-shoulder-p-tq (list 0) l-shoulder-y-tq (list 0)               
        ;; l-elbow-p-tq (list 0) l-wrist-r-tq (list 0) l-wrist-p-tq (list 0) l-wrist-y-tq (list 0) 
        ;; r-shoulder-r-tq (list 0) r-shoulder-p-tq (list 0) r-shoulder-y-tq (list 0)               
        ;; r-elbow-p-tq (list 0) r-wrist-r-tq (list 0) r-wrist-p-tq (list 0) r-wrist-y-tq (list 0)

        ;; link-name-list '(:rleg-link0 :rleg-link1 :rleg-link2 :rleg-link3 :rleg-link4 :rleg-link5 :rleg-link6
        ;;                              :lleg-link0 :lleg-link1 :lleg-link2 :lleg-link3 :lleg-link4 :lleg-link5 :lleg-link6
        ;;                              :torso-link0 :torso-link1 :head-link0 :head-link1
        ;;                              :rarm-link0 :rarm-link1 :rarm-link2 :rarm-link3 :rarm-link4 :rarm-link5 :rarm-link6
        ;;                              :larm-link0 :larm-link1 :larm-link2 :larm-link3 :larm-link4 :larm-link5 :larm-link6)

        link-name-list '(6 7 8 11 14 13 9   15 16 17 20 23 22 9   9 9 42 44   24 25 26 27 32 31 30   33 34 35 36 41 40 39)

        ))

;; gnuplot設定
(defun gnuplot-setting (gnuplot-instance &key xtics ytics)
  (if xtics ;; 目盛り幅設定
      (send gnuplot-instance :command (format nil "set xtics ~a" xtics))
    (send gnuplot-instance :command "unset xtics"))
  (if ytics ;; 目盛り幅設定
      (send gnuplot-instance :command (format nil "set ytics ~a" ytics))
    (send gnuplot-instance :command "unset ytics"))
  (send gnuplot-instance :command "set xlabel \"time[s]\"")
  (send gnuplot-instance :command "set ylabel \"joint angular velocity [rad/sec]\"")
  )

;; 関節角速度プロット
(defun plot-angle-velocity (limb cur-tm tm-span tm-range idx-range offset-tm)
  (let* ( (idx (send mframe :idx))
         (av0 (send (elt mframe-list-origin (max 0 (- idx 3))) :angle-vector))
         (av1 (send (elt mframe-list-origin (max 0 (- idx 2))) :angle-vector))
         (av2 (send (elt mframe-list-origin (max 0 (1-  idx))) :angle-vector))
         (av3 (send (elt mframe-list-origin idx) :angle-vector))
         (velvec0 (scale (/ 1 tm-span) (vector-deg2rad (v- av1 av0))));; rad/sec
         (velvec1 (scale (/ 1 tm-span) (vector-deg2rad (v- av2 av1))));; rad/sec
         (velvec2 (scale (/ 1 tm-span) (vector-deg2rad (v- av3 av2))));; rad/sec
         (velvec (scale (/ 1.0 3) (reduce #'v+ (list velvec0 velvec1 velvec2))))
         (list-length (length r-crotch-y-vel))
         )
    (push-each-list (send *m* :model) velvec link-name-list
                    (list r-crotch-y-vel r-crotch-r-vel r-crotch-p-vel r-knee-p-vel r-ankle-p-vel r-ankle-r-vel r-toe-p-vel
                          l-crotch-y-vel l-crotch-r-vel l-crotch-p-vel l-knee-p-vel l-ankle-p-vel l-ankle-r-vel l-toe-p-vel
                          waist-y-vel waist-p-vel neck-y-vel neck-p-vel
                          r-shoulder-p-vel r-shoulder-r-vel r-shoulder-y-vel r-elbow-p-vel r-wrist-y-vel r-wrist-r-vel r-wrist-p-vel 
                          l-shoulder-p-vel l-shoulder-r-vel l-shoulder-y-vel l-elbow-p-vel l-wrist-y-vel l-wrist-r-vel l-wrist-p-vel ))
    (cond
     ( (equal limb :leg)
       (send *gnuplot*
             :draw (subseq r-crotch-y-vel (- list-length idx-range))
             (subseq r-crotch-r-vel (- list-length idx-range))
             (subseq r-crotch-p-vel (- list-length idx-range))
             (subseq r-knee-p-vel (- list-length idx-range))
             (subseq r-ankle-p-vel (- list-length idx-range))
             (subseq r-ankle-r-vel (- list-length idx-range))
             (subseq r-toe-p-vel (- list-length idx-range))
             :xoffset (- cur-tm (min (* list-length tm-span) tm-range))
             :xscale tm-span
             :xrange (list (+ cur-tm (- offset-tm) (- tm-range)) (+ cur-tm (- offset-tm)))
             :yrange (list -7 7)
             :title (list "r-crotch-y-vel" "r-crotch-r-vel" "r-crotch-p-vel" "r-knee-p-vel" "r-ankle-p-vel" "r-ankle-r-vel" "r-toe-p-vel"))
       (send *gnuplot1*
             :draw (subseq l-crotch-y-vel (- list-length idx-range))
             (subseq l-crotch-r-vel (- list-length idx-range))
             (subseq l-crotch-p-vel (- list-length idx-range))
             (subseq l-knee-p-vel (- list-length idx-range))
             (subseq l-ankle-p-vel (- list-length idx-range))
             (subseq l-ankle-r-vel (- list-length idx-range))
             (subseq r-toe-p-vel (- list-length idx-range))
             :xoffset (- cur-tm (min (* list-length tm-span) tm-range))
             :xscale tm-span
             :xrange (list (+ cur-tm (- offset-tm) (- tm-range)) (+ cur-tm (- offset-tm)))
             :yrange (list -7 7)
             :title (list "l-crotch-y-vel" "l-crotch-r-vel" "l-crotch-p-vel" "l-knee-p-vel" "l-ankle-p-vel" "l-ankle-r-vel" "l-toe-p-vel"))
       )
     ( (equal limb :arm)
       (send *gnuplot*
             :draw (subseq r-shoulder-p-vel (- list-length idx-range))
             (subseq r-shoulder-r-vel (- list-length idx-range))
             (subseq r-shoulder-y-vel (- list-length idx-range))
             (subseq r-elbow-p-vel (- list-length idx-range))
             (subseq r-wrist-y-vel (- list-length idx-range))
             (subseq r-wrist-r-vel (- list-length idx-range))
             (subseq r-wrist-p-vel (- list-length idx-range))
             :xoffset (- cur-tm (min (* list-length tm-span) tm-range))
             :xscale tm-span
             :xrange (list (+ cur-tm (- offset-tm) (- tm-range)) (+ cur-tm (- offset-tm)))
             :yrange (list -15 15)
             :title (list "r-shoulder-p-vel" "r-shoulder-r-vel" "r-shoulder-y-vel" "r-elbow-p-vel" "r-wrist-y-vel" "r-wrist-r-vel" "r-wrist-p-vel"))
       (send *gnuplot1*
             :draw (subseq l-shoulder-p-vel (- list-length idx-range))
             (subseq l-shoulder-r-vel (- list-length idx-range))
             (subseq l-shoulder-y-vel (- list-length idx-range))
             (subseq l-elbow-p-vel (- list-length idx-range))
             (subseq l-wrist-y-vel (- list-length idx-range))
             (subseq l-wrist-r-vel (- list-length idx-range))
             (subseq l-wrist-p-vel (- list-length idx-range))
             :xoffset (- cur-tm (min (* list-length tm-span) tm-range))
             :xscale tm-span
             :xrange (list (+ cur-tm (- offset-tm) (- tm-range)) (+ cur-tm (- offset-tm)))
             :yrange (list -15 15)
             :title (list "l-shoulder-p-vel" "l-shoulder-r-vel" "l-shoulder-y-vel" "l-elbow-p-vel" "l-wrist-y-vel" "l-wrist-r-vel" "l-wrist-p-vel"))
       ) )
    ))

;; bvhファイルから各種リスト作成
(defun make-lists
  (&key  (fname "walk") (ik t) (copy-pose t) (animate t) (smooth t)
         (span 1) tm-span idx-range (tm-range 2) (offset-tm 0) (z-thresh 70) (max-frame 500) (slow-play nil) (start-frame 0) (z-offset 0)
         (xtics 0.5) (ytics 2);; gnuplot目盛り
         (vel-plot nil) )
  (let ( (relpos #f(0 0 0)) (robot-z-offset 0) (model-z-offset 0) (av) (fnum 0) bvhfile ret-list
         ;; animation )
         )
    (setq *motionname* fname)

    ;; 時刻系統初期設定
    (if tm-span   (setq span  (round (/ tm-span  0.03))) (setq tm-span (* span 0.03)));; 2プロット間の時刻
    (if idx-range (setq tm-range (* idx-range tm-span)) (setq idx-range (round (/ tm-range tm-span))));; プロットする時刻の幅
    (setq *ft* (* 0.03 span));; frame time (tm-spanと同じ)

    (setq bvhfile (concatenate string (unix::getenv "HOME") "/" (unix::getenv "BVHDIR") "/" *motionname* ".bvh") );;bvhファイル読み込み

    (setq *waist-z-offset* nil)
    (setq *start-frame* start-frame)

    ;; 各種リスト宣言
    (declare-lists)
    (setq mframe-list-origin nil
          rframe-list-origin nil)

    (gnuplot-setting *gnuplot* :xtics xtics :ytics ytics);; gnuplot設定

    ;; モnデル作成
    (cond
     ((equal *motionname* "nocchi");;perfume-bvh
      (setq *m* (load-mcd bvhfile :scale 10 :bvh-robot-model-class perfume-bvh-robot-model))
      (setq model-z-offset 100));; モデルごとの微妙な高さの差の調整
     (t;;kinect-bvh
      ;; (setq *m* (load-mcd bvhfile :scale 1 :bvh-robot-model-class kinect-bvh-robot-model))
      (setq *m* (load-mcd bvhfile :scale 1 :bvh-robot-model-class kinect-sdk-sub-bvh-robot-model))
      (setq model-z-offset 0)))

    (setq animation-origin (send *m* :animation))
    ;; (setq animation (do-kalman-filter animation-origin :r-variance 0.005))
    (setq animation (do-kalman-filter animation-origin :r-variance 0.001))
    ;; (setq animation (do-kalman-filter animation-origin :r-variance 0.0001))

    ;; モデル情報の初期化 これより前にモデルを動かしてはならない
    (init-model-param animation)

    (cond ((substringp  "HRP2JSK" (send *robot* :name)) (setq robot-z-offset 250));; robotごとの高さの調整
          ((equal (send *robot* :name) "HRP4R") (setq robot-z-offset 1050))
          ((equal (send *robot* :name) "P1") (setq robot-z-offset 80)))
 
    (send *m* :frame 0)
    (if copy-pose
        ;; (objects (list (send *m* :model) *robot* *box* *lfoot* *rfoot* *a*))
        ;; (objects (list (send *m* :model) *robot* *box* *lfoot* *rfoot* ))
        (objects (list (send *m* :model) *box* *lfoot* *rfoot* ))
      (objects (list (send *m* :model) *box* )))

    ;; loop to make list
    (block :animate
      (dotimes (idx (send *m* :frame-length))

        (when (= (mod idx span) 0)

          ;;状態更新
          (format stdout "bvh frame ~A  time ~A  pseq ~A~%" idx (* *ft* idx) fnum)
          (send *m* :frame idx)

          ;; userが見つかった時のみ
          (when (and (not (equal (send (car (send *m* :model :torso)) :worldpos)  #f(0 0 0) )) (< (+ idx *start-frame*) (send *m* :frame-length)) )
            (incf fnum)
            (send *m* :frame (+ idx *start-frame*))
            (send *m* :model :set-joint-angle (elt animation (+ idx *start-frame*)));; 関節角セット
            
            (setq mframe (instance frame :init)
                  rframe (instance frame :init))

            (send mframe :set-idx (+ idx *start-frame*));; *start-frame*を足す意味はあるのか?
            (send mframe :set-time (* *ft* (send mframe :idx)))
            (send rframe :set-idx (+ idx *start-frame*))
            (send rframe :set-time (* *ft* (send rframe :idx)))

            (if copy-pose (send *m* :model :copy-state-to *robot*))

            ;; ロボットが宙に浮かないように
            ;; bvhとrobotのtorsoの設定位置の違いを修正
            (setq relpos (v- (send (elt (send *m* :model :torso) 0) :worldpos)
                             (send (send *robot* :root) :worldpos)))
            ;; bvhのtorsoのデフォルト高さの修正
            (setf (elt relpos 2) (- (elt relpos 2) model-z-offset robot-z-offset))
            (send *robot* :translate relpos :world)

            (when (equal (send *robot* :name) "P1")
              (setq av (send *robot* :angle-vector))
              (send *robot* :angle-vector
                    (concatenate float-vector (subseq av 0 12) #f(-70.0 90.0 90.0 -20.0 90.0 0.0 0.0 0.0 70.0 90.0 -90.0 -20.0 -90.0 0.0 0.0 0.0))))


            ;; 各種リスト作成
            ;; bvhから直接取得 （爪先ではなく足首）
            (setq lfoot-vec (v- (subseq (elt animation (+ idx start-frame)) 15 18) (subseq (elt animation (+ idx start-frame)) 12 15) )
                  rfoot-vec (v- (subseq (elt animation (+ idx start-frame)) 27 30) (subseq (elt animation (+ idx start-frame)) 24 27) ) )

            (send mframe :set-coord :lleg (make-coords :pos (subseq (elt animation (send mframe :idx)) 12 15)
                                                       :rot (calc-rot lfoot-vec) ))
            (send mframe :set-coord :rleg (make-coords :pos (subseq (elt animation (send mframe :idx)) 24 27)
                                                       :rot (calc-rot rfoot-vec) ))
            (send mframe :set-coord :larm (send (send (elt (send *m* :model :rarm) 2) :worldcoords) :copy-worldcoords));; 左右逆
            (send mframe :set-coord :rarm (send (send (elt (send *m* :model :larm) 2) :worldcoords) :copy-worldcoords))

            ;; joint-pos-list
            (send mframe :set-joint-pos-list-from-av (elt animation (send mframe :idx)))


            (setq ret-list (copy-leg-state (elt animation (send mframe :idx)) :z-offset z-offset));; waist-coord lfoot-coord rfoot-coord を返す
            (send rframe :set-coord :waist (elt ret-list 0))
            (send rframe :set-coord :lleg (elt ret-list 1))
            (send rframe :set-coord :rleg (elt ret-list 2))

            (send mframe :set-angle-vector (send *m* :model :angle-vector))
            (send rframe :set-angle-vector (send *robot* :angle-vector))

            (push-back (copy-object mframe) mframe-list-origin)
            (push-back (copy-object rframe) rframe-list-origin)

            ;; 人の関節角速度プロット
            (when vel-plot
              (plot-angle-velocity :leg (send mframe :time) tm-span tm-range idx-range offset-tm) )
              ;; (plot-angle-velocity ':arm (send mframe :time) tm-span tm-range idx-range offset-tm) )

            (when slow-play (unix:usleep (round (* 1000 1000 *ft*))))
            ;; (when animate (send *pickview* :draw-objects :flush nil)
            (send *pickview* :look-all)
                  );; 描画

            ;; (send (make-coords :pos (scale 1000 (subseq (send *m* :model :angle-vector) 0 3))
            ;;                    :rpy (mapcar #'deg2rad (coerce (subseq (send *m* :model :angle-vector) 3 6) cons))) :draw-on :flush t :size 300)

            (x::window-main-one)
            (if (or (select-stream (list *standard-input*) 1.0e-8) (> fnum max-frame)) (return-from :animate nil))

            ))
        )) ;; end animate 

    (warn "wh ~a  wr ~a  pitch ~a" *wh* *wr* *pitch*)
    ))

;; フィルタリング
;; vel-threshは7くらいでもいい？？？
(defun do-filter (&key (animate nil) (stdout nil) (steal t) (vertical t) (raise t) (avg t) (free t) (footprint t)  (keypose t) (oldbvh nil)
                       (z-thresh 100) (pos-thresh 100) (vel-thresh 15) (term 4)  (scale 1) (touch-thresh 0.25) (draw-thresh 0.35)
                       (point-angle-thresh 30) (point-z-thresh 175) (point-vel-thresh 20) (yawvel-thresh 0.05) (yawangle-thresh 1) )
  (let ( (floor 0) return-list listfile listpath )

    (objects (list (send *m* :model) *robot* *box* *lfoot* *rfoot* ))

    ;; 股関節yaw開き最大角度
    (setq *crotch-yaw-max-angle* (deg2rad (- (* 2 (abs (send *robot* :lleg :crotch-y :max-angle))) 5)));; 5度のマージン

    ;; 床高さ調節
    (cond
     (( equal (send *robot* :name) "HRP2JSK"))
     (( equal (send *robot* :name) "P1") (setq floor 3.5))
     )

    ;; (warn "z-thresh ~a  vel-thresh ~a  pos-thresh ~a~%point-angle-thresh ~a  point-z-thresh ~a  yawvel-thresh ~a  yawangle-thresh ~a~%touch-thresh ~a  draw-thresh ~a~% floor ~a ~%"
    ;;       z-thresh vel-thresh pos-thresh point-angle-thresh point-z-thresh yawvel-thresh yawangle-thresh touch-thresh draw-thresh floor)

    ;; リスト記録ファイル
    (setq listpath (concatenate string (unix::getenv "HOME") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" *motionname*) )
    (if (not (probe-file listpath)) (unix::system (concatenate string "mkdir " listpath )));; ディレクトリがない場合
    (setq listfile (concatenate string listpath "/" *motionname* ".l") )

    ;; 人間zmp計算
    ;; start-frameの分を考えていない
    ;; (setq zmp-list (calc-zmp-from-skeleton-animation (smoothfilter (send *m* :animation) :mod-list '(1 1 1 1 1 1 1)) :old oldbvh))

    ;; smooth
    ;; (setq zmp-list (smoothfilter zmp-list :mod-list '(1 1 1)));; zmp

    ;; smooth
    ;; ;; model
    ;; (setq mframe-list (smoothfilter mframe-list-origin :target-slot :lfoot-coord :mod-list '(1 1 1 1 1));; coord
    ;;       mframe-list (smoothfilter mframe-list        :target-slot :rfoot-coord :mod-list '(1 1 1 1 1))
    ;;       mframe-list (smoothfilter mframe-list        :target-slot :lhand-coord :mod-list '(1 1 1 1 1))
    ;;       mframe-list (smoothfilter mframe-list        :target-slot :rhand-coord :mod-list '(1 1 1 1 1))
    ;;       ;; mframe-list (smoothfilter (calc-vel-from-pos mframe-list :waist) :target-slot :waist-vel  :mod-list '(1 1 1 1 1));; vel
    ;;       mframe-list (smoothfilter (calc-vel-from-pos mframe-list :lleg) :target-slot :lfoot-vel   :mod-list '(1 1 1))
    ;;       mframe-list (smoothfilter (calc-vel-from-pos mframe-list :rleg) :target-slot :rfoot-vel   :mod-list '(1 1 1))
    ;;       mframe-list (smoothfilter (calc-vel-from-pos mframe-list :larm) :target-slot :lhand-vel   :mod-list '(1 1 1 1 1))
    ;;       mframe-list (smoothfilter (calc-vel-from-pos mframe-list :rarm) :target-slot :rhand-vel   :mod-list '(1 1 1 1 1))
    ;;       mframe-list (smoothfilter (calc-yawvel-from-rot mframe-list :lleg) :target-slot :lfoot-yawvel   :mod-list '(1 1 1 1 1));; yawvel
    ;;       mframe-list (smoothfilter (calc-yawvel-from-rot mframe-list :rleg) :target-slot :rfoot-yawvel   :mod-list '(1 1 1 1 1))
    ;;       mframe-list (smoothfilter mframe-list        :target-slot :joint-pos-list :mod-list '(1 1 1 1 1 1 1));; joint-pos-list
    ;;       mframe-list (smoothfilter (calc-zmp mframe-list :old oldbvh) :target-slot :zmp :mod-list '(1 1 1));; zmp
    ;;       )
    ;; ;; robot
    ;; (setq rframe-list (smoothfilter rframe-list-origin :target-slot :waist-coord :mod-list '(1 1 1 1 1));; coord
    ;;       rframe-list (smoothfilter rframe-list        :target-slot :lfoot-coord :mod-list '(1 1 1))
    ;;       rframe-list (smoothfilter rframe-list        :target-slot :rfoot-coord :mod-list '(1 1 1)) )

    ;; calc vel,yawvel,zmp
    (setq mframe-list (calc-vel-from-pos mframe-list-origin :lleg)
          mframe-list (calc-vel-from-pos mframe-list :rleg)
          mframe-list (calc-vel-from-pos mframe-list :larm)
          mframe-list (calc-vel-from-pos mframe-list :rarm)
          mframe-list (calc-yawvel-from-rot mframe-list :lleg)
          mframe-list (calc-yawvel-from-rot mframe-list :rleg)
          mframe-list (calc-zmp mframe-list :old oldbvh) )

    (setq rframe-list (copy-object rframe-list-origin))

    
    (setq lz-list-origin nil)(dolist (val mframe-list) (push-back (send val :z :lleg) lz-list-origin))
    (setq rz-list-origin nil)(dolist (val mframe-list) (push-back (send val :z :rleg) rz-list-origin))
    (setq lvel-list-origin nil)(dolist (val mframe-list) (push-back (send val :vel :lleg) lvel-list-origin))
    (setq rvel-list-origin nil)(dolist (val mframe-list) (push-back (send val :vel :rleg) rvel-list-origin))
    (setq lyawvel-list-origin nil)(dolist (val mframe-list) (push-back (send val :yawvel :lleg) lyawvel-list-origin))
    (setq ryawvel-list-origin nil)(dolist (val mframe-list) (push-back (send val :yawvel :rleg) ryawvel-list-origin))
    (setq zmpy-list-origin nil)(dolist (val mframe-list) (push-back (elt (send val :zmp) 1) zmpy-list-origin))

    ;; kalman-filter
    (setq mframe-list (do-kalman-filter mframe-list :target-slot :lfoot-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :rfoot-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :lhand-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :rhand-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :lfoot-yawvel)
          mframe-list (do-kalman-filter mframe-list :target-slot :rfoot-yawvel)
          mframe-list (do-kalman-filter mframe-list :target-slot :zmp) )


    (warn "end kalman~%")

    ;; 接地状態識別
    (setq mframe-list (check-foot-state mframe-list :z-thresh z-thresh :vel-thresh vel-thresh :pos-thresh pos-thresh
                                        :yawvel-thresh yawvel-thresh :yawangle-thresh yawangle-thresh 
                                        :point-angle-thresh point-angle-thresh :point-z-thresh point-z-thresh :point-vel-thresh point-vel-thresh
                                        :touch-thresh touch-thresh :draw-thresh draw-thresh :term term))

    ;; (setq lstate-list-tmp2 nil)(dolist (val mframe-list) (push-back (send val :state :lleg) lstate-list-tmp2))
    ;; (setq rstate-list-tmp2 nil)(dolist (val mframe-list) (push-back (send val :state :rleg) rstate-list-tmp2))

    ;; mframeからrframeに接地状態をコピー
    (setq rframe-list (copy-frame-slot mframe-list rframe-list '(:vel :state) '(:lleg :rleg :larm :rarm)))

    (setq lstate-list-medi0 nil)(dolist (val mframe-list) (push-back (send val :state :lleg) lstate-list-medi0))
    (setq rstate-list-medi0 nil)(dolist (val mframe-list) (push-back (send val :state :rleg) rstate-list-medi0))

    (setq lz-list-medi0 nil)(dolist (val mframe-list) (push-back (send val :z :lleg) lz-list-medi0))
    (setq rz-list-medi0 nil)(dolist (val mframe-list) (push-back (send val :z :rleg) rz-list-medi0))
    (setq lvel-list-medi0 nil)(dolist (val mframe-list) (push-back (send val :vel :lleg) lvel-list-medi0))
    (setq rvel-list-medi0 nil)(dolist (val mframe-list) (push-back (send val :vel :rleg) rvel-list-medi0))
    (setq lyawvel-list-medi0 nil)(dolist (val mframe-list) (push-back (send val :yawvel :lleg) lyawvel-list-medi0))
    (setq ryawvel-list-medi0 nil)(dolist (val mframe-list) (push-back (send val :yawvel :rleg) ryawvel-list-medi0))
    (setq zmpy-list-medi0 nil)(dolist (val mframe-list) (push-back (elt (send val :zmp) 1) zmpy-list-medi0))

    (setq robot-z-list-medi0 nil)(dolist (val rframe-list) (push-back (elt  (send val :waist-pos) 2) robot-z-list-medi0))
    (setq robot-lz-list-medi0 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 2) robot-lz-list-medi0))
    (setq robot-rz-list-medi0 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 2) robot-rz-list-medi0))
    (setq robot-ly-list-medi0 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 1) robot-ly-list-medi0))
    (setq robot-ry-list-medi0 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 1) robot-ry-list-medi0))
    (setq robot-lpitch-list-medi0 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :lfoot-coord))) robot-lpitch-list-medi0))
    (setq robot-rpitch-list-medi0 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :rfoot-coord))) robot-rpitch-list-medi0))

    ;; 特定動作動作のみ平滑化
    ;; (setq rframe-list (smoothfilter lcoord-list :mod-list '(1 1 1 1 1 1 1 1 1 1 1) :z-list lz-list :state *point*)
    ;;       rframe-list (smoothfilter rcoord-list :mod-list '(1 1 1 1 1 1 1 1 1 1 1) :z-list rz-list :state *point*) )

    ;; キーポーズ抽出
    (if keypose (setq rframe-list (extract-keyframe rframe-list)))

    ;; 歩幅調整
    (setq rframe-list (horizontal-scale rframe-list scale))
    (warn "end horisontal scale~%")


    (catch :filter

      ;; 底上げ 接地区間決定のために地面下の目標位置は除去
      (when raise (setq rframe-list (raise-bottom rframe-list :lleg) rframe-list (raise-bottom rframe-list :rleg))
            (warn "end raise bottom~%") )

      (setq lstate-list-medi1 nil)(dolist (val rframe-list) (push-back (send val :state :lleg) lstate-list-medi1))
      (setq rstate-list-medi1 nil)(dolist (val rframe-list) (push-back (send val :state :rleg) rstate-list-medi1))

      (setq lz-list-medi1 nil)(dolist (val rframe-list) (push-back (send val :z :lleg) lz-list-medi1))
      (setq rz-list-medi1 nil)(dolist (val rframe-list) (push-back (send val :z :rleg) rz-list-medi1))
      (setq lvel-list-medi1 nil)(dolist (val mframe-list) (push-back (send val :vel :lleg) lvel-list-medi1))
      (setq rvel-list-medi1 nil)(dolist (val mframe-list) (push-back (send val :vel :rleg) rvel-list-medi1))

      (setq robot-z-list-medi1 nil) (dolist (val rframe-list) (push-back (elt  (send val :waist-pos) 2) robot-z-list-medi1))
      (setq robot-lz-list-medi1 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 2) robot-lz-list-medi1))
      (setq robot-rz-list-medi1 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 2) robot-rz-list-medi1))
      (setq robot-ly-list-medi1 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 1) robot-ly-list-medi1))
      (setq robot-ry-list-medi1 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 1) robot-ry-list-medi1))
      (setq robot-lpitch-list-medi1 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :lfoot-coord))) robot-lpitch-list-medi1))
      (setq robot-rpitch-list-medi1 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :rfoot-coord))) robot-rpitch-list-medi1))


      ;; 忍び足(z衝撃緩和) 接地区間決定
      (when (and steal (not keypose))
        ;; (setq return-list (stealfilter lcoord-list lz-list lvel-list :floor floor))
        ;; (setq lcoord-list (car return-list)  lz-list (cadr return-list))
        ;; (setq return-list (stealfilter rcoord-list rz-list rvel-list :floor floor))
        ;; (setq rcoord-list (car return-list)  rz-list (cadr return-list))
        (setq rframe-list (stealfilter rframe-list :lleg :floor floor) rframe-list (stealfilter rframe-list :rleg :floor floor))
        (warn "end steal~%") )

      (setq robot-z-list-medi2 nil) (dolist (val rframe-list) (push-back (elt  (send val :waist-pos) 2) robot-z-list-medi2))
      (setq robot-lz-list-medi2 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 2) robot-lz-list-medi2))
      (setq robot-rz-list-medi2 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 2) robot-rz-list-medi2))
      (setq robot-ly-list-medi2 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 1) robot-ly-list-medi2))
      (setq robot-ry-list-medi2 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 1) robot-ry-list-medi2))
      (setq robot-lpitch-list-medi2 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :lfoot-coord))) robot-lpitch-list-medi2))
      (setq robot-rpitch-list-medi2 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :rfoot-coord))) robot-rpitch-list-medi2))


      ;; avg 足裏フィッティング
      (when avg (setq rframe-list (foot-fit rframe-list :lleg :floor floor) rframe-list (foot-fit rframe-list :rleg :floor floor))
            (warn "end avg~%") )

      (setq robot-z-list-medi3 nil) (dolist (val rframe-list) (push-back (elt  (send val :waist-pos) 2) robot-z-list-medi3))
      (setq robot-lz-list-medi3 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 2) robot-lz-list-medi3))
      (setq robot-rz-list-medi3 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 2) robot-rz-list-medi3))
      (setq robot-ly-list-medi3 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 1) robot-ly-list-medi3))
      (setq robot-ry-list-medi3 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 1) robot-ry-list-medi3))
      (setq robot-lpitch-list-medi3 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :lfoot-coord))) robot-lpitch-list-medi3))
      (setq robot-rpitch-list-medi3 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :rfoot-coord))) robot-rpitch-list-medi3))


      (send *gnuplot* :draw
            lstate-list-medi0 rstate-list-medi0
            ;; robot-lz-list-medi1 robot-rz-list-medi1
            lvel-list-medi0 rvel-list-medi0)

      ;;足跡決定 ik fail 足干渉 解消
      (cond
       ( footprint
         (setq rframe-list (decide-footprint rframe-list :animate animate :stdout_ stdout :floor floor))
         (setq rframe-list-medi (copy-seq rframe-list))
         (dump-loadable-structure listfile rframe-list rframe-list-medi mframe-list *ft* *start-frame*) ;; リスト記録
         (warn "end foot print~%") )
       ( t (load listfile)
           ;; ここで一旦 footprintを飛ばした時もそうでない時も中間生成ファイルを使う
           (setq rframe-list (copy-seq rframe-list-medi)) ) )

      ;; 腰高さのギャップ緩衝
      ;; (setq coord-list (moderate-gap coord-list :thresh 2))

      (setq robot-z-list-medi4 nil) (dolist (val rframe-list) (push-back (elt  (send val :waist-pos) 2) robot-z-list-medi4))
      (setq robot-lz-list-medi4 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 2) robot-lz-list-medi4))
      (setq robot-rz-list-medi4 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 2) robot-rz-list-medi4))
      (setq robot-ly-list-medi4 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 1) robot-ly-list-medi4))
      (setq robot-ry-list-medi4 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 1) robot-ry-list-medi4))
      (setq robot-lpitch-list-medi4 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :lfoot-coord))) robot-lpitch-list-medi4))
      (setq robot-rpitch-list-medi4 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :rfoot-coord))) robot-rpitch-list-medi4))

      ;; 垂直接地
      (when (and vertical (not keypose))
        (setq rframe-list (verticalfilter rframe-list :lleg)  rframe-list (verticalfilter rframe-list :rleg))
        (warn "end vertical~%") )
      

      ;; 衝撃緩和軌道 ik-fail 除去
      ;; (setq coord-list (decide-non-inpact coord-list lcoord-list rcoord-list lz-list rz-list))

      (setq robot-z-list-medi5 nil) (dolist (val rframe-list) (push-back (elt  (send val :waist-pos) 2) robot-z-list-medi5))
      (setq robot-lz-list-medi5 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 2) robot-lz-list-medi5))
      (setq robot-rz-list-medi5 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 2) robot-rz-list-medi5))
      (setq robot-ly-list-medi5 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 1) robot-ly-list-medi5))
      (setq robot-ry-list-medi5 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 1) robot-ry-list-medi5))
      (setq robot-lpitch-list-medi5 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :lfoot-coord))) robot-lpitch-list-medi5))
      (setq robot-rpitch-list-medi5 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :rfoot-coord))) robot-rpitch-list-medi5))


      ;; 遊脚軌道
      (cond
       ( free
         ;; 地面食い込み ik-fail 自己干渉 除去
         (setq rframe-list (decide-free-leg rframe-list :lleg :animate animate))
         (setq rframe-list (decide-free-leg rframe-list :rleg :animate animate))
         (warn "end free leg~%")

         ;; 短期的ik-fail除去
         ;; (setq lcoord-list (remove-short-ik-fail-seq coord-list lcoord-list lz-list :lleg))
         ;; (setq rcoord-list (remove-short-ik-fail-seq coord-list rcoord-list rz-list :rleg))
         (dump-loadable-structure listfile rframe-list-medi rframe-list mframe-list *ft* *start-frame*);; リスト記録
         (warn "end short free~%") )
       ( (not footprint) )
       ( t  (load listfile) ) )


      );; end chatch filter

    (setq robot-z-list-medi6 nil) (dolist (val rframe-list) (push-back (elt  (send val :waist-pos) 2) robot-z-list-medi6))
    (setq robot-lz-list-medi6 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 2) robot-lz-list-medi6))
    (setq robot-rz-list-medi6 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 2) robot-rz-list-medi6))
    (setq robot-ly-list-medi6 nil)(dolist (val rframe-list) (push-back (elt  (send val :lfoot-pos) 1) robot-ly-list-medi6))
    (setq robot-ry-list-medi6 nil)(dolist (val rframe-list) (push-back (elt  (send val :rfoot-pos) 1) robot-ry-list-medi6))
    (setq robot-lpitch-list-medi6 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :lfoot-coord))) robot-lpitch-list-medi6))
    (setq robot-rpitch-list-medi6 nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :rfoot-coord))) robot-rpitch-list-medi6))


    (send *gnuplot* :draw
          lstate-list-medi0 rstate-list-medi0
          robot-lz-list-medi6 robot-rz-list-medi6
          lvel-list-medi0 rvel-list-medi0
          )

    nil))


;; 各種リストからchoreonoidポーズ列を作成
(defun make-pseq
  (&key (ik t) (copy t) (animate t) (write nil) (span 3) (max-pseq nil) (zmp-offset 0) (speed 1) (exception nil) (all-frame nil)
        direction (couple nil) )
  (let ( pseqfile (fnum 0) ik-fail-flg-cons ik-fail-flg idx-diff animation prev-idx next-idx (dt 0) )
    (setq pseqfile (concatenate string *motionname* ".pseq"))
    (setq pseqpath (concatenate string (unix::getenv "HOME") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" *motionname* "/"))

    ;; (setq last-touch-flg (list t t))

    (if (null max-pseq) (setq max-pseq (length rframe-list)))

    (setq animation (send *m* :animation))

    (objects (list (send *m* :model) *robot* *box* *lfoot* *rfoot*))


    (setq robot-lpitch-list nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :lfoot-coord))) robot-lpitch-list))
    (setq robot-rpitch-list nil)(dotimes (piv (length rframe-list)) (push-back (rad2deg (calc-pitch (send (elt rframe-list piv) :rfoot-coord))) robot-rpitch-list))


    (setq robot-ly-list nil robot-ry-list nil  robot-lz-list nil  robot-rz-list nil);; リスト宣言

    (if (not (probe-file pseqpath)) (unix::system (concatenate string "mkdir " pseqpath )));; ディレクトリがない場合
    (if write (setq direction :output) (setq direction :input))
    (with-open-file
     (fp (concatenate string pseqpath pseqfile) :direction direction :if-does-not-exist :create)

     (when write
       ;;pseqファイルの冒頭
       (format fp (concatenate string
                               "# Body pose sequence format version 1.0 defined by cnoid-Robotics~%~%"
                               "type: PoseSeq~%"
                               "name: " "\"" *motionname* "\"~%"
                               "targetBody: \"" (send *robot* :name) "\"~%"
                               "refs: ~%"
                               ))
       ;; 初期動作
       (setq *touch-flg* (list t t))
       (send *robot* :reset-pose)
       (send *robot* :fix-leg-to-coords (send (elt rframe-list 0) :coord :lleg) :lleg)
       (format fp (write-buf 1 :init t)) ;; sec
       (format fp (write-buf 1))
       (setq *touch-flg* (list t nil))
       (send *robot* :rleg :move-end-pos #f(0 0 50))
       (format fp (write-buf 1))
       )

     ;; 表示 loop
     (block :animate
       (dotimes (idx (length rframe-list))
         (setq idx-diff 0  ik-fail-flg t)
         
         (format stdout "time ~A   pseq frame ~A~%" (* *ft* idx) idx)
         ;; (when (not (equal (send (car (send *m* :model :torso)) :worldpos)  #f(0 0 0) ));; userが見つかった時のみ

         (while ik-fail-flg;; ik-failだと前のフレームで埋める
           (setq ik-fail-flg nil)
           (send *robot* :move-to (send (elt rframe-list (+ idx idx-diff)) :coord :waist) :world)
           (send *robot* :translate (float-vector 0 0 zmp-offset) :world);; zmp補償のためのオフセット

           (send *robot* :angle-vector (send (elt rframe-list (+ idx idx-diff)) :angle-vector))

           (when couple
             (send *robot* :larm :angle-vector #f(-30.0 10.0 0.0 -50.0 0.0 0.0 -10.0 -15.0) )
             (send *robot* :rarm :angle-vector #f(-30.0 10.0 0.0 -50.0 0.0 0.0 -10.0 -15.0) ) )

           ;; (send *m* :model :angle-vector
           ;;       (concatenate float-vector (scale 1000 (subseq (elt model-av-list (+ idx idx-diff)) 0 3)) (subseq (elt model-av-list (+ idx idx-diff)) 3)))
           ;; (send *m* :frame (+ idx idx-diff)) ;; bvhfileにpositionが入っているからこれではだめ
           (send *m* :model :set-joint-angle (elt animation (+ idx idx-diff *start-frame*)));; 関節角セット

           (setq ik-fail-flg-cons
                 (exec-foot-ik (+ idx idx-diff) rframe-list
                               (list (list t t 2 "    left ik failed" )(list :xy :z 3 " z+y failed")(list :z :z 3 " xy+rp failed")(list :z nil 1 " z failed"))
                               (list (list t t 4 "    right ik failed")(list :xy :z 3 " z+y failed")(list :z :z 3 " xy+rp failed")(list :z nil 1 " z failed"))) )
           (when (or (> (car ik-fail-flg-cons) 0) (> (cdr ik-fail-flg-cons) 0) ) (setq ik-fail-flg t) (incf idx-diff) )
           (if ik-fail-flg (warn " ~a" idx-diff))
           );; end ik fail loop
         (if (> idx-diff 0)(warn "~%"))

         (if (= (pqp-collision-check (car (last (send *robot* :lleg :links))) (car (last (send *robot* :rleg :links)))) 1)
             (warning-message 1 " leg-leg ~a" (+ idx idx-diff)) )
         (if (and (not (= (send (elt rframe-list idx) :state :lleg) *ground*)) (= (pqp-collision-check (car (last (send *robot* :lleg :links))) *box*) 1) )
             (warning-message 1 " box-lleg ~a" (+ idx idx-diff)) )
         (if (and (not (= (send (elt rframe-list idx) :state :rleg) *ground*)) (= (pqp-collision-check (car (last (send *robot* :rleg :links))) *box*) 1) )
             (warning-message 1 " box-rleg ~a" (+ idx idx-diff)) )
             

         (when animate (send *pickview* :draw-objects)
               ;; (unix:usleep (round (* 1000 *ft*)))
               (unix:usleep (* 1000 100))
               );; (send *pickview* :look-all));; 描画

         ;; 変換後リスト作成
         (push-back (elt (send *robot* :lleg :end-coords :worldpos) 1) robot-ly-list)
         (push-back (elt (send *robot* :rleg :end-coords :worldpos) 1) robot-ry-list)
         (push-back (elt (send *robot* :lleg :end-coords :worldpos) 2) robot-lz-list)
         (push-back (elt (send *robot* :rleg :end-coords :worldpos) 2) robot-rz-list)

         (cond;; 接地足フラグ
          ((and (= (send (elt rframe-list idx) :state :lleg) 0) (= (send (elt rframe-list idx) :state :rleg) 0) ) (setq *touch-flg* (list t t)) );; 両足fit
          ((= (send (elt rframe-list idx) :state :lleg) 0) (setq *touch-flg* (list t nil)) );;左足fit
          ((= (send (elt rframe-list idx) :state :rleg) 0) (setq *touch-flg* (list nil t)) );;右足fit
          (t (setq *touch-flg* (list nil nil)) );; 跳躍
          )

         ;; (setq prev-idx (max 0 (1- idx))  next-idx (min (1- (length rframe-list)) (1+ idx)))
         ;; (setq dt (+ dt *ft*))
         ;; (when (and write
         ;;            (or (and exception
         ;;                     (some #'eval (mapcar #'(lambda (limb) (and (= (send (elt rframe-list idx) :state limb) 0) 
         ;;                                                                (or (not (= (send (elt rframe-list prev-idx) :state limb) 0)) (not (= (send (elt rframe-list next-idx) :state limb) 0)))))
         ;;                                          (list :lleg :rleg)) ));; 直前直後が非接地で現在が接地の場合はspanにかかわらず記録する
         ;;                (= (mod idx span) 0 ) ) )

         ;;   ;; 直前直後が接地で現在が非接地の場合は記録しない
         ;;   (setq prev-idx (max 0 (- idx span))  next-idx (min (1- (length rframe-list)) (+ idx span)))
         ;;   (when (or all-frame
         ;;             (not (some #'eval (mapcar #'(lambda (limb) (and (not (= (send (elt rframe-list idx) :state limb) 0))
         ;;                                                             (or (= (send (elt rframe-list prev-idx) :state limb) 0) (= (send (elt rframe-list next-idx) :state limb) 0))))
         ;;                                 (list :lleg :rleg)) )) )
             
         ;;     ;; 遊脚高さが一定以下の時は記録しない
         ;;     (when (not (or (and (not (car *touch-flg*)) (< (send (elt rframe-list idx) :z :lleg) 5))
         ;;                    (and (not (cadr *touch-flg*)) (< (send (elt rcoord-list idx) :z :rleg) 5)) ) )
         ;;       ;;バッファ作成
         ;;       (if (= idx 0)
         ;;           (format fp (write-buf 1))
         ;;         (format fp (write-buf dt  :speed speed))
         ;;         )
         ;;       (setq dt 0) ;; dtリセット
         ;;       )
         ;;     (incf fnum)
         ;;     )

         ;;   )
         (when write
           (if (= idx 0)
               (format fp (write-buf 1))
             (format fp (write-buf (- (send (elt rframe-list idx) :time) (send (elt rframe-list (1- idx)) :time))  :speed speed :keypose-flg-list (send (elt rframe-list idx) :get-keypose-flg-list) ))
             ;; (format fp (write-buf (- (send (elt rframe-list idx) :time) (send (elt rframe-list (1- idx)) :time))  :speed speed  :keypose-flg-list (list :lleg :rleg :larm :rarm)))
             )
           )

         ;; );; when not end (success detect user)
         
         (when (or (select-stream (list *standard-input*) 1.0e-8) (> fnum max-pseq))(return-from :animate nil))
         (x::window-main-one)

         );; end dotimes
       );; end block

     ;; 終端動作
     (when write
       (format fp (write-buf 1))
       (setq *touch-flg* (list t nil))
       (send *robot* :rleg :move-end-pos #f(0 0 50))
       (format fp (write-buf 1)) ;; sec
       (setq *touch-flg* (list t t))
       (send *robot* :reset-pose)
       (send *robot* :fix-leg-to-coords (send (car (last rframe-list )) :coord :lleg) :lleg)
       (format fp (write-buf 1))
       (format fp (write-buf 1))
       )
     
     );; with-open-file

    ;; gnuplot
    (send *gnuplot* :draw
          lstate-list-medi1 rstate-list-medi1 robot-lz-list robot-rz-list
          ;; robot-lpitch-list robot-rpitch-list
          ;; :xrange (list 0 (length robot-lz-list))
          ;; :yrange (list -1 1)
          :title '("lz-list" "rz-list" "robot lleg" "robot rleg")
          ;; :title '( "left p"  "right p")
          )

    ))

;; write-bufのoffset-vecを決めるときなど
;; reset-pose時 の足や腰の座標を手動で作ったpseqファイルと比較
(defun av-to-pseq
  (&key (ik t) (copy t) (animate t) )
  (let ( pseqfile pos pseqpath (init t) keypose-flg-lists waist-coord-list )
    (setq *motionname* "angle-vector")
    (setq pseqfile (concatenate string *motionname* ".pseq"))
    (setq pseqpath (concatenate string (unix::getenv "HOME") "/" (unix::getenv "CNOID_WORKSPACE") "/" (send *robot* :name) "/" *motionname* "/"))

    ;;pseqファイルの冒頭
    (setq buf (concatenate string
                           "# Body pose sequence format version 1.0 defined by cnoid-Robotics~%~%"
                           "type: PoseSeq~%"
                           "name: " "\"" "angle-vector" "\"~%"
                           "targetBody: \"" (send *robot* :name) "\"~%"
                           "refs: ~%"
                           ))

    (objects (list *robot* *box*))

    (cond
     ( (equal (send *robot* :name) "HRP2JSK")
       (setq pos #f(2250 0 648))
       (setq waist-coord-list
             (list (make-coords :pos pos) (make-coords :pos pos) (make-coords :pos pos)))
       (setq av-list 
             (list #f(0.0 0.0 -26.0 50.0 -24.0 0.0
                          0.0 0.0 -26.0 50.0 -24.0 0.0
                          0.0 0.0 0.0 0.0
                          10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0
                          10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0)
                   #f(0.0 0.0 -26.0 50.0 -24.0 0.0
                          0.0 0.0 -50.0 80.0 -24.0 0.0
                          0.0 0.0 0.0 0.0
                          10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0
                          10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0)
                   #f(0.0 0.0 -26.0 50.0 -24.0 0.0
                          0.0 0.0 -26.0 50.0 -24.0 0.0
                          0.0 0.0 0.0 0.0
                          10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0
                          10.0 10.0 0.0 -25.0 0.0 0.0 -10.0 -15.0)
                   ))
       (setq keypose-flg-lists (list '(:lleg :rleg :larm :rarm) '(:lleg) '(:lleg :rleg :larm :rarm))) )
     ( (equal (send *robot* :name) "HRP4R") (setq pos #f(2250 0 -42))
       (setq av-list
             (list #f(0.0 -1.0 -20.0 40.0 -20.0 1.0 0.0 1.0 -20.0 40.0 -20.0 -1.0   8.0 0.0 0.0 0.0
                          -3.0 -10.0 0.0 -30.0 0.0 0.0 0.0 1.0 0.0 -3.0 10.0 0.0 -30.0 0.0 0.0 0.0 -1.0 0.0)
                   #f(0.0 -1.0 -20.0 40.0 -20.0 1.0 0.0 1.0 -40.0 60.0 -20.0 -1.0   8.0 0.0 0.0 0.0
                          -3.0 -10.0 0.0 -30.0 0.0 0.0 0.0 1.0 0.0 -3.0 10.0 0.0 -30.0 0.0 0.0 0.0 -1.0 0.0)
                   #f(0.0 -1.0 -20.0 40.0 -20.0 1.0 0.0 1.0 -20.0 40.0 -20.0 -1.0   8.0 0.0 0.0 0.0
                          -3.0 -10.0 0.0 -30.0 0.0 0.0 0.0 1.0 0.0 -3.0 10.0 0.0 -30.0 0.0 0.0 0.0 -1.0 0.0)
                   #f(0.0 -1.0 -40.0 60.0 -20.0 1.0 0.0 1.0 -20.0 40.0 -20.0 -1.0   8.0 0.0 0.0 0.0
                          -3.0 -10.0 0.0 -30.0 0.0 0.0 0.0 1.0 0.0 -3.0 10.0 0.0 -30.0 0.0 0.0 0.0 -1.0 0.0)
                   )) )
     ( (equal (send *robot* :name) "P1") (setq pos #f(2250 0 944)))
     (t (setq pos #f(2250 0 0))))

    (send *robot* :locate pos :world)(send *robot* :rpy 0 0 0)

    ;;ファイル書き込み
    (when copy
      ;; (if (not (probe-file pseqpath)) (mkdir ,pseqpath))
      (with-open-file
       (fp (concatenate string pseqpath pseqfile) :direction :output)
       
       (format fp buf)
       ;; foot fit loop
       (let ( (j 0) (ft_  1)  lz rz)
         (while (< j (length av-list))
           (send *robot* :move-to (elt waist-coord-list j) :world)
           (send *robot* :angle-vector (elt av-list j))
           (setq lz (elt (send *robot* :lleg :end-coords :worldpos) 2))
           (setq rz (elt (send *robot* :rleg :end-coords :worldpos) 2))

           (cond;; 接地足フラグ
            ((and (< lz 3) (< rz 3) ) (setq *touch-flg* (list t t)) );; 両足fit
            ((< lz 3) (setq *touch-flg* (list t nil)) );;左足fit
            ((< rz 3) (setq *touch-flg* (list nil t)) );;右足fit
            )

           (format stdout "time ~A   pseq frame ~A~%" (* ft_ j) j)

           (when animate (send *pickview* :draw-objects));; (send *pickview* :look-all));; 描画

           ;;バッファ作成
           (if init
               (progn
                 (format fp (write-buf 1 :init t))
                 (setq init nil) )
             (format fp (write-buf 1 :keypose-flg-list (elt keypose-flg-lists j))) )

           (unix:usleep (round (* 1000 1000 ft_)))
           (if (select-stream (list *standard-input*) 1.0e-8) (throw :animate nil))
           (x::window-main-one)
           (incf j)
           );; end while
         );; end catch
       ))

    ))


(defun test ()
  ;; (setq *m* (load-mcd "bvhfiles/nocchi.bvh" :scale 10
  ;;                     :bvh-robot-model-class perfume-bvh-robot-model))
  ;; (setq f #f(0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
  ;;                0.0 0.0 0.0 30.0 30.0 -30.0  50.0 0.0 0.0  40.0 30.0 30.0
  ;;                0.0 0.0 0.0 -30.0 30.0 30.0  -50.0 0.0 0.0  -40.0 30.0 -30.0
  ;;                0.0 -20.0 -10.0  0.0 20.0 0.0  0.0 0.0 0.0  0.0 0.0 0.0
  ;;                0.0 -20.0  10.0  0.0 20.0 0.0  0.0 0.0 0.0  0.0 0.0 0.0))

  (setq *m* (load-mcd "bvhfiles/walk.bvh" :scale 1
                      :bvh-robot-model-class kinect-bvh-robot-model))
  (setq f #f(0.0 0.0 0.0 0.0 0.0 0.0
               30.0 30.0 -30.0  0.0 -30.0 0.0
               -30.0 30.0 30.0  0.0 -30.0 0.0
               30.0 20.0 40.0  0.0 0.0 40.0
               -30.0 20.0 -40.0  0.0 0.0 -40.0
               0.0 0.0 0.0))

  (setq *model* (send *m* :model))
  (objects (list *model* *robot*))


  (send *model* :angle-vector f)
  (send *m* :model :copy-state-to *robot*)

  ;; (setq relpos (v- (send (elt (send *m* :model :torso) 1) :worldpos)
  ;;                  (send (elt (send *robot* :torso) 0) :worldpos)))

  )


;; 前回保存しておいたmotion listの読み込み
(defun load-motion (&key (fname  "SampleMotion") )
  (let (bvhfile)
    (setq *motionname* fname)
    (warn "load ~a~%" *motionname*)
    (setq bvhfile (concatenate string (unix::getenv "HOME") "/prog/euslib/demo/k-kojima/bvh/bvhfiles/" *motionname* ".bvh") )

    
    (load (format nil "~a/~a/~a/~a/~a.l" (unix::getenv "HOME") (unix::getenv "CNOID_WORKSPACE") (send *robot* :name) *motionname* *motionname*))
    (make-lists :fname *motionname* :max-frame (1- (length coord-list)) :animate nil :start-frame *start-frame*)
    (load (format nil "~a/~a/~a/~a/~a.l" (unix::getenv "HOME") (unix::getenv "CNOID_WORKSPACE") (send *robot* :name) *motionname* *motionname*))

    (cond
     ( (equal *motionname* "nochi") (setq *m* (load-mcd bvhfile :scale 10 :bvh-robot-model-class perfume-bvh-robot-model)) )
     ( t (setq *m* (load-mcd bvhfile :scale 1 :bvh-robot-model-class  kinect-sdk-sub-bvh-robot-model)) ) )

    (objects (list (send *m* :model) *robot* *box* *lfoot* *rfoot* ))

  ))