;; sdkのbvhに合わせて棒人間モデルが動く
#!/usr/bin/env roseus
#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(ros::roseus "sdk-bvh2eus")
(load "euslib/rbrain/eusbvh.l")
(load "euslib/demo/k-kojima/bvh/kinect-sdk-sub-model-viewer.l")
(load "package://roseus_tutorials/src/kinect-model-viewer.l")
(load "euslib/demo/k-kojima/bvh/filter.l")
(load "euslib/demo/k-kojima/bvh/frame-class.l")

(load "euslib/jsk/gnuplotlib.l")

(defvar font-helvetica-bold-34 (x::font-id "-*-Helvetica-Bold-R-Normal-*-34-*"))
(defvar font-helvetica-34 (x::font-id "-*-Helvetica-Medium-R-Normal-*-34-*"))

(if (not (boundp '*pickview*)) (pickview :no-menu t))
(send *pickview* :resize 800 600)
(send *pickview* :whiteback t)

(if (not (boundp 'stdout)) (setq stdout nil))

(if (not (boundp '*robot*)) (setq *robot* (hrp2jsk-simple-detail)))

(if (not (boundp '*floor*)) (setq *floor* (make-cube 1000 1000 1)))
(send *floor* :locate #f(2500 -250 0) :world)
(send *floor* :set-color :gray)

(if (not (boundp '*keypose*)) (setq *keypose* nil))

(when (not (boundp '*gnuplot*)) (setq *gnuplot* (gnuplot))(send *gnuplot* :command "set grid"))

;; 腰・足関節表示用オブジェクト
(when (not (boundp '*waist*))
  (setq *waist* (make-cube 10 150 300))
  ;; (send *waist* :locate #f(80 0 100))
  ;; (setq *waist* (body+ (make-cube 10 300 500) *waist*))
  ;; (send *waist* :locate #f(0 0 0))
  )
(when (not (boundp 'obj-list))
  (setq obj-list nil)
  (dotimes (i 16)
    (setq obj-list (append obj-list (list (make-cube 50 50 50))))
    (send (elt obj-list i) :set-color :red)) )

;; (when (not (boundp 'regression-obj-list))
;;   (setq regression-obj-list nil)
;;   (dotimes (i 8)
;;     (setq regression-obj-list (append regression-obj-list (list (make-cube 50 50 50))))
;;     (send (elt regression-obj-list i) :set-color :blue)) )

;; リンク表示用オブジェクト
(when (not (boundp 'link-obj-list))
  (setq link-obj-list nil)
  (dotimes (i 14)
    (setq link-obj-list (append link-obj-list (list (make-cube 50 50 50))))
    (send (elt link-obj-list i) :set-color :yellow)) )


(if (not (boundp '*zmp*)) (setq *zmp* (make-cube 50 50 50)))
(send *zmp* :set-color :green)

(setq *ft* 0.03)

(defun init-model-param()
  (let (x y_ x_ (foot-vec #f(0 0 0)))
    ;; 足面ピッチ角度計算 最初の10frameの平均
    (dotimes (i 10)
      (setq foot-vec (v+ foot-vec
                         (v- (subseq (elt animation i) 15 18) (subseq (elt animation i) 12 15))
                         (v- (subseq (elt animation i) 27 30) (subseq (elt animation i) 24 27))))
      )
    (setq x (normalize-vector foot-vec))
    (setq y_ (normalize-vector (v* #f(0 0 1) x)))
    (setq x_ (normalize-vector (v* y_ #f(0 0 1))))
    (setq *pitch*  (rad2deg (acos (v. x_ x))))

    ))

;; リアルタイムに接地状態を識別 check-leg-stateに変更を反映
;; (defun edit-state-list (z vel pre-state leg-pos-list lr &key  (vel-thresh 10) (z-thresh 100) (point-angle-thresh 20) (point-z-thresh 175))
;;   (let (x y_ x_ theta foot-vec tmp-list)
;;     (setq tmp (eval z))

;;     ;; 足ベクトルの水平面との角度計算
;;     (setq foot-vec (v- (elt leg-pos-list  3) (elt leg-pos-list 2)))
;;     (setq x (normalize-vector foot-vec))
;;     (setq y_ (normalize-vector (v* #f(0 0 1) x)))
;;     (setq x_ (normalize-vector (v* y_ #f(0 0 1))))
;;     (setq theta  (- (acos (v. x_ x)) (deg2rad *pitch*)))
;;     (cond
;;      ( (equal lr :lleg) (push (rad2deg theta) lp-list) )
;;      ( (equal lr :rleg) (push (rad2deg theta) rp-list) ))

;;     (cond
;;      ;; 接地またはヒール・トーポイント
;;      ( (and (< vel vel-thresh) (< z point-z-thresh))
;;        (cond
;;         ;; ヒール・トーポイント
;;         ( (and (not (eps= (rad2deg theta) 0  point-angle-thresh)) (not (= pre-state 0)) )
;;           (setq tmp 50) )
;;         ;; 面接地
;;         ( (< z z-thresh)
;;           (setq tmp 0) ) ) )

;;      ;; すり足
;;      ( (< z z-thresh)
;;        (setq tmp 100))
;;      )

;;     tmp
;;     ))

(defun play (&key (fname "test") (animate t) (foot-touch-state-show t)
                  (z-thresh 100)  (pos-thresh 100) (vel-thresh 15);; (vel-thresh 10)
                  (point-z-thresh 175) (point-angle-thresh 30) (point-vel-thresh 20) ;; (point-angle-thresh 20)
                  (term 4)
                  (touch-thresh 0.25) (draw-thresh 0.35) (yawvel-thresh 0.05) (yawangle-thresh 1)
                  (old nil) )
  (let ( (move-targets nil) (link-lists nil)  target-pos av-list lfoot-vec rfoot-vec (keypose-idx 0)
         lleg-pos-list rleg-pos-list larm-pos-list rarm-pos-list
         (bvhfile (concatenate string (unix::getenv "HOME") "/" (unix::getenv "BVHDIR") "/" fname ".bvh") ) )
    ;; (warn "z-thresh ~a  vel-thresh ~a  pos-thresh ~a~%point-angle-thresh ~a  point-z-thresh ~a  yawvel-thresh ~a  yawangle-thresh ~a~%touch-thresh ~a  draw-thresh ~a~% floor ~a ~%"
    ;;       z-thresh vel-thresh pos-thresh point-angle-thresh point-z-thresh yawvel-thresh yawangle-thresh touch-thresh draw-thresh floor)

    (setq *motionname* fname)
    (setq *m* (load-mcd bvhfile :scale 1
                        :bvh-robot-model-class kinect-sdk-sub-bvh-robot-model))

    (setq animation (send *m* :animation))
    ;; (setq av-list (smoothfilter (send *m* :animation) :mod-list '(1 1 1 1 1)));; av-listの3から6要素はとってはならない
    (setq av-list (do-kalman-filter animation :r-variance 0.001))
    (setq animation (do-kalman-filter animation :r-variance 0.001))


    (init-model-param);; 足面オフセットピッチ角度計算

    ;; 各種リスト宣言
    (setq mframe-list-origin nil
          diff-list-tmp nil
          diff-list nil)

    ;; リスト作成
    (dotimes (i (length animation))
      (setq mframe (instance frame :init))
      ;; index time
      (send mframe :set-idx i)
      (send mframe :set-time (* *ft* (send mframe :idx)))

      (send *m* :model :set-joint-angle (elt animation (send mframe :idx)));; 棒人間モデルの関節角セット

      ;; foot-vec
      (setq lfoot-vec (v- (subseq (elt animation i) 15 18) (subseq (elt animation i) 12 15) )
            rfoot-vec (v- (subseq (elt animation i) 27 30) (subseq (elt animation i) 24 27) ) )
      ;; coord
      (send mframe :set-coord :lleg (make-coords :pos (subseq (elt animation (send mframe :idx)) 12 15)
                                                 :rot (calc-rot lfoot-vec) ))
      (send mframe :set-coord :rleg (make-coords :pos (subseq (elt animation (send mframe :idx)) 24 27)
                                                 :rot (calc-rot rfoot-vec) ))
      (send mframe :set-coord :larm (send (send (elt (send *m* :model :rarm) 2) :worldcoords) :copy-worldcoords))
      (send mframe :set-coord :rarm (send (send (elt (send *m* :model :larm) 2) :worldcoords) :copy-worldcoords))
      (send mframe :set-coord :lleg-toe (make-coords :pos (subseq (elt animation (send mframe :idx)) 15 18)
                                                 :rot (calc-rot lfoot-vec) ))
      (send mframe :set-coord :rleg-toe (make-coords :pos (subseq (elt animation (send mframe :idx)) 27 30)
                                                 :rot (calc-rot rfoot-vec) ))
      ;; angle-vector
      (send mframe :set-angle-vector (send *m* :model :angle-vector))

      ;; joint-pos-list
      (send mframe :set-joint-pos-list-from-av (elt animation i))

      (push-back (copy-object mframe) mframe-list-origin)
      )

    ;; zmp計算
    ;; (setq zmp-list-origin (calc-zmp-from-skeleton-animation (smoothfilter animation :mod-list '(1 1 1 1 1 1 1) ) :old old))

    ;; 平滑化
    ;; (setq zmp-list (smoothfilter zmp-list-origin :mod-list '(1 1 1)));; zmp
    ;; (setq mframe-list (smoothfilter mframe-list-origin :target-slot :lfoot-coord :mod-list '(1 1 1 1 1 1));; coord
    ;;       mframe-list (smoothfilter mframe-list        :target-slot :rfoot-coord :mod-list '(1 1 1 1 1 1))
    ;;       mframe-list (smoothfilter mframe-list        :target-slot :lhand-coord :mod-list '(1 1 1 1 1 1))
    ;;       mframe-list (smoothfilter mframe-list        :target-slot :rhand-coord :mod-list '(1 1 1 1 1 1))
    ;;       ;; mframe-list (smoothfilter (calc-vel-from-pos mframe-list :waist) :target-slot :waist-vel  :mod-list '(1 1 1 1 1));; vel
    ;;       mframe-list (smoothfilter (calc-vel-from-pos mframe-list :lleg) :target-slot :lfoot-vel   :mod-list '(1 1 1))
    ;;       mframe-list (smoothfilter (calc-vel-from-pos mframe-list :rleg) :target-slot :rfoot-vel   :mod-list '(1 1 1))
    ;;       mframe-list (smoothfilter (calc-vel-from-pos mframe-list :larm) :target-slot :lhand-vel   :mod-list '(1 1 1 1 1))
    ;;       mframe-list (smoothfilter (calc-vel-from-pos mframe-list :rarm) :target-slot :rhand-vel   :mod-list '(1 1 1 1 1))
    ;;       mframe-list (smoothfilter (calc-yawvel-from-rot mframe-list :lleg) :target-slot :lfoot-yawvel   :mod-list '(1 1 1 1 1));; yawvel
    ;;       mframe-list (smoothfilter (calc-yawvel-from-rot mframe-list :rleg) :target-slot :rfoot-yawvel   :mod-list '(1 1 1 1 1))
    ;;       mframe-list (smoothfilter mframe-list        :target-slot :joint-pos-list :mod-list '(1 1 1 1 1 1 1));; joint-pos-list
    ;;       mframe-list (smoothfilter (calc-zmp mframe-list :old old) :target-slot :zmp :mod-list '(1 1 1));; zmp
    ;;       )
    (setq mframe-list (calc-vel-from-pos mframe-list-origin :lleg)
          mframe-list (calc-vel-from-pos mframe-list :rleg)
          mframe-list (calc-vel-from-pos mframe-list :larm)
          mframe-list (calc-vel-from-pos mframe-list :rarm)
          mframe-list (calc-vel-from-pos mframe-list :lleg-toe)
          mframe-list (calc-vel-from-pos mframe-list :rleg-toe)
          mframe-list (calc-yawvel-from-rot mframe-list :lleg)
          mframe-list (calc-yawvel-from-rot mframe-list :rleg)
          mframe-list (calc-zmp mframe-list :old old) )
    ;; kalman-filter
    (setq mframe-list (do-kalman-filter mframe-list :target-slot :lfoot-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :rfoot-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :ltoe-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :rtoe-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :lhand-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :rhand-vel)
          mframe-list (do-kalman-filter mframe-list :target-slot :lfoot-yawvel)
          mframe-list (do-kalman-filter mframe-list :target-slot :rfoot-yawvel)
          mframe-list (do-kalman-filter mframe-list :target-slot :zmp)
          )


    ;; 接地状態認識
    (setq mframe-list (check-foot-state mframe-list :z-thresh z-thresh :vel-thresh vel-thresh :pos-thresh pos-thresh
                                        :yawvel-thresh yawvel-thresh :yawangle-thresh yawangle-thresh :point-angle-thresh point-angle-thresh :point-z-thresh point-z-thresh
                                        :touch-thresh touch-thresh :draw-thresh draw-thresh :term term))

    (setq lstate-list nil)(dolist (val mframe-list) (push-back (send val :state :lleg) lstate-list))
    (setq rstate-list nil)(dolist (val mframe-list) (push-back (send val :state :rleg) rstate-list))
    (setq lz-list nil)(dolist (val mframe-list) (push-back (send val :z :lleg) lz-list))
    (setq rz-list nil)(dolist (val mframe-list) (push-back (send val :z :rleg) rz-list))
    (setq lvel-list nil)(dolist (val mframe-list) (push-back (send val :vel :lleg) lvel-list))
    (setq rvel-list nil)(dolist (val mframe-list) (push-back (send val :vel :rleg) rvel-list))
    (setq ltoevel-list nil)(dolist (val mframe-list) (push-back (send val :vel :lleg-toe) ltoevel-list))
    (setq rtoevel-list nil)(dolist (val mframe-list) (push-back (send val :vel :rleg-toe) rtoevel-list))
    (setq lhandvel-list nil)(dolist (val mframe-list) (push-back (send val :vel :larm) lhandvel-list))
    (setq rhandvel-list nil)(dolist (val mframe-list) (push-back (send val :vel :rarm) rhandvel-list))
    (setq lpitch-list nil)(dolist (val mframe-list) (push-back (rad2deg (calc-pitch (send val :coord :lleg))) lpitch-list))
    (setq rpitch-list nil)(dolist (val mframe-list) (push-back (rad2deg (calc-pitch (send val :coord :rleg))) rpitch-list))
    (setq lyawvel-list nil)(dolist (val mframe-list) (push-back (rad2deg (send val :yawvel :lleg)) lyawvel-list))
    (setq ryawvel-list nil)(dolist (val mframe-list) (push-back (rad2deg (send val :yawvel :rleg)) ryawvel-list))

    ;; 接地状態遷移出力
    (with-open-file
     (fp (concatenate string (unix::getenv "HOME") "/Dropbox/foot-touch-state.txt") :direction :output :if-does-not-exist :create)
     (let (lstate rstate)
       (dotimes (i (length lstate-list))
         (cond
          ( (= (elt lstate-list i) *ground*) (setq lstate 0) )
          ( (= (elt lstate-list i) *touch*)  (setq lstate 1) )
          ( (= (elt lstate-list i) *point*)  (setq lstate 2) )
          ( (= (elt lstate-list i) *fan*)    (setq lstate 3) )
          ( (= (elt lstate-list i) *slide*)  (setq lstate 4) )
          ( (= (elt lstate-list i) *draw*)   (setq lstate 5) )
          ( (= (elt lstate-list i) *float*)  (setq lstate 6) ) )
         (cond
          ( (= (elt rstate-list i) *ground*) (setq rstate 0) )
          ( (= (elt rstate-list i) *touch*)  (setq rstate 1) )
          ( (= (elt rstate-list i) *point*)  (setq rstate 2) )
          ( (= (elt rstate-list i) *fan*)    (setq rstate 3) )
          ( (= (elt rstate-list i) *slide*)  (setq rstate 4) )
          ( (= (elt rstate-list i) *draw*)   (setq rstate 5) )
          ( (= (elt rstate-list i) *float*)  (setq rstate 6) ) )
         (format fp "~a,~a,~a~%" (/ i 30.0) lstate rstate)
       )))

    ;; キーポーズ抽出
    (setq mkeyframe-list (extract-keyframe mframe-list))

    ;; (objects (append (list  (send *m* :model) (make-cube 100 100 100) *waist* *robot* *floor* ) obj-list))
    ;; (objects (flatten (list  (send *m* :model) *floor* *waist* *zmp* obj-list)))
    ;; (objects (flatten (list  (send *m* :model) *floor*  *zmp* obj-list regression-obj-list)))
    (objects (flatten (list  (send *m* :model) *floor*  *zmp* obj-list)))
    ;; (objects (list (send *m* :model) *floor*))
    ;; (objects (append (list  *floor* ) obj-list))

    (catch :animate
      (dotimes (i (send *m* :frame-length))
      ;; (dotimes (i 1)
      ;;   (setq i frame)
        (send *m* :frame i);; 全体位置・姿勢セット

        ;; 脚関節の長さ 400 180 150
        ;; まずhip,shoulderの位置を出力するように変更
        (setq av (elt av-list i))

        ;; (setq av #f(0 0 0  0 0 0
        ;;               0 100 -100   300 200 -500  300 200 -780  450 250 -780
        ;;               ;; 0  100 -100  -100  190 -600   -200  290 -980   -100 290 -980
        ;;               0 -100 -100  -100 -140 -600  -200 -190 -980  -100 -190 -980
        ;;               0  420 190  0  680 190  0  680 140
        ;;               0 -420 190  0 -680 190  0 -680 140
        ;;               0 0 400))
        ;; (send *m* :model :angle-vector av)
                      
        (setq lleg-pos-list (list (subseq av 6 9) (subseq av 9 12) (subseq av 12 15) (subseq av 15 18)));; 左足
        (setq rleg-pos-list (list (subseq av 18 21) (subseq av 21 24) (subseq av 24 27) (subseq av 27 30)));; 右足
        ;; 脚関節オブジェクト移動
        (dotimes (j 4)
          (send (elt obj-list j) :locate (elt lleg-pos-list j) :world)
          (send (elt obj-list (+ j 4)) :locate (elt rleg-pos-list j) :world) )

        (setq larm-pos-list (list (subseq av 30 33) (subseq av 33 36) (subseq av 36 39) (subseq av 39 42)));; 左足
        (setq rarm-pos-list (list (subseq av 42 45) (subseq av 45 48) (subseq av 48 51) (subseq av 51 54)));; 右足
        ;; 手関節オブジェクト移動
        (dotimes (j 4)
          (send (elt obj-list (+ 8 j)) :locate (elt larm-pos-list j) :world)
          (send (elt obj-list (+ 8 j 4)) :locate (elt rarm-pos-list j) :world) )

        (when foot-touch-state-show
          (send *pickview* :viewer :viewsurface :color #f(0 0.7 0))

          (send *pickview* :viewer :viewsurface :string 40 50 "Float" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 40 100 "Draw" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 40 150 "Slide" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 40 200 "Fan" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 40 250 "Point" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 40 300 "Touch" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 40 350 "Full-Grounded" font-helvetica-bold-34)

          (send *pickview* :viewer :viewsurface :string 560 50 "Float" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 560 100 "Draw" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 560 150 "Slide" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 560 200 "Fan" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 560 250 "Point" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 560 300 "Touch" font-helvetica-bold-34)
          (send *pickview* :viewer :viewsurface :string 560 350 "Full-Grounded" font-helvetica-bold-34)

          (send *pickview* :viewer :viewsurface :color #f(1 0 0))
          (cond;; 左足
           ( (= (elt lstate-list i) *ground*) 
             (send (elt obj-list 2) :set-color :black)
             (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(530 320) 30 30)
             (send *pickview* :viewer :viewsurface :string 560 350 "Full-Grounded" font-helvetica-bold-34) )
           ( (= (elt lstate-list i) *touch*) 
             (send (elt obj-list 2) :set-color :orange)
             (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(530 270) 30 30)
             (send *pickview* :viewer :viewsurface :string 560 300 "Touch" font-helvetica-bold-34) )
           ( (= (elt lstate-list i) *point*)
             (send (elt obj-list 2) :set-color :yellow)
             (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(530 220) 30 30)
             (send *pickview* :viewer :viewsurface :string 560 250 "Point" font-helvetica-bold-34) )
           ( (= (elt lstate-list i) *fan*)
             (send (elt obj-list 2) :set-color :pink)
             (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(530 170) 30 30)
             (send *pickview* :viewer :viewsurface :string 560 200 "Fan" font-helvetica-bold-34) )
           ( (= (elt lstate-list i) *slide*)
             (send (elt obj-list 2) :set-color :blue)
             (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(530 120) 30 30)
             (send *pickview* :viewer :viewsurface :string 560 150 "Slide" font-helvetica-bold-34) )
           ( (= (elt lstate-list i) *draw*)
             (send (elt obj-list 2) :set-color :green)
             (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(530 70) 30 30)
             (send *pickview* :viewer :viewsurface :string 560 100 "Draw" font-helvetica-bold-34) )
           (t (send (elt obj-list 2) :set-color :red)
              (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(530 20) 30 30)
              (send *pickview* :viewer :viewsurface :string 560 50 "Float" font-helvetica-bold-34) )
           )(send (elt obj-list 0) :set-color :red)
            (cond;; 右足
             ( (= (elt rstate-list i) *ground*)
               (send (elt obj-list 6) :set-color :black) 
               (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(10 320) 30 30)
               (send *pickview* :viewer :viewsurface :string 40 350 "Full-Grounded" font-helvetica-bold-34) )
             ( (= (elt rstate-list i) *touch*)
               (send (elt obj-list 6) :set-color :orange)
               (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(10 270) 30 30)
               (send *pickview* :viewer :viewsurface :string 40 300 "Touch" font-helvetica-bold-34) )
             ( (= (elt rstate-list i) *point*) 
               (send (elt obj-list 6) :set-color :yellow)
               (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(10 220) 30 30)
               (send *pickview* :viewer :viewsurface :string 40 250 "Point" font-helvetica-bold-34) )
             ( (= (elt rstate-list i) *fan*) 
               (send (elt obj-list 6) :set-color :pink)
               (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(10 170) 30 30)
               (send *pickview* :viewer :viewsurface :string 40 200 "Fan" font-helvetica-bold-34) )
             ( (= (elt rstate-list i) *slide*) 
               (send (elt obj-list 6) :set-color :blue)
               (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(10 120) 30 30)
               (send *pickview* :viewer :viewsurface :string 40 150 "Slide" font-helvetica-bold-34) )
             ( (= (elt rstate-list i) *draw*) 
               (send (elt obj-list 6) :set-color :green)
               (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(10 70) 30 30)
               (send *pickview* :viewer :viewsurface :string 40 100 "Draw" font-helvetica-bold-34) )
             (t (send (elt obj-list 6) :set-color :red)
                (send *pickview* :viewer :viewsurface :draw-fill-rectangle #f(10 20) 30 30)
                (send *pickview* :viewer :viewsurface :string 40 50 "Float" font-helvetica-bold-34) )
             )
            (send (elt obj-list 4) :set-color :red)
            )
        (send *pickview* :viewer :viewsurface :flush)


        ;; 腰オブジェクト移動
        ;; (send *waist* :move-to (make-coords :pos (subseq (elt animation  i) 0 3) :rpy (subseq (elt animation i) 3 6)) :world)
        (send *waist* :move-to (make-coords :pos (subseq (elt animation  i) 0 3) ) :world)
        (if (and (not (= (elt lstate-list i) 0))  (not (= (elt rstate-list i) 0)))
            (progn (send *waist* :set-color :red) (warn " ~a jump  " i))
          (send *waist* :set-color :blue))
        (send (elt obj-list 0) :set-color :red)

        (send *zmp* :locate (send (elt mframe-list i) :zmp) :world);; zmpオブジェクト移動

        ;; 棒人間モデルの姿勢設定(米倉式)
        ;; (send *m* :model :set-joint-angle (concatenate float-vector (subseq (elt animation i) 0 6) (subseq (elt av-list i) 6)) :type :yonekura)
        ;; (push-back (calc-joint-pos-diff (append lleg-pos-list rleg-pos-list)) diff-list-tmp)

        ;; 棒人間モデルの姿勢設定
        (send *m* :model :set-joint-angle (concatenate float-vector (subseq (elt animation i) 0 6) (subseq (elt av-list i) 6)))

        ;; 棒人間モデルとskeletonデータの関節位置の差分
        ;; (push-back (calc-joint-pos-diff (append lleg-pos-list rleg-pos-list)) diff-list)

        ;; キーポーズ表示
        (cond
         ( (and *keypose* (eps= (send (elt mframe-list i) :time) (send (elt mkeyframe-list (min keypose-idx (1- (length mkeyframe-list)))) :time) 0.1))
           (send *floor* :set-color :red)
           (if (not (eps= (send (elt mframe-list (min (1+ i) (1- (length mframe-list)))) :time) (send (elt mkeyframe-list (min keypose-idx (1- (length mkeyframe-list)))) :time) 0.1))
               (incf keypose-idx) ) )
         ( t (send *floor* :set-color :gray) )
         )

        ;; (send *m* :model :copy-state-to *robot*)

        (when animate
            (if (= i 0) (send (get *viewer* :pickviewer) :look-all) (send *pickview* :draw-objects :flush nil));; 表示範囲調整
            (unix::usleep (* 1000 33))
            ;; (unix::usleep (* 1000 33)) 
            (if (select-stream (list *standard-input*) 1.0e-8) (throw :animate nil))
            (x::window-main-one)
            ;; (print (send (elt mframe-list i) :time))
            )
        );; end dotimes
      );; end animate

    nil
    ))

;; 棒人間モデルとskeletonデータの関節位置の差分
(defun calc-joint-pos-diff (skeleton-pos-list)
  (let (model-coord-list)
    (setq model-coord-list (flatten (send *m* :model :legs)))
    (apply #'+ (mapcar #'(lambda (model-coord skeleton-pos) (norm (v- (send model-coord :worldpos) skeleton-pos))) model-coord-list skeleton-pos-list))
    ))
    


;; z-thresh
;; v-thresh

;; point-angle-thresh
;; yawvel-thresh

;; draw-thresh
;; touch-thresh

;; 閾値決定
(defun decide-thresh ( state  low high step
                              &key (fname "test") (frame 0) (animate t) (old nil) (term nil) )
  (let ( (z-thresh 100) (vel-thresh 10) (point-z-thresh 175) (point-angle-thresh 20) 
         (touch-thresh 0.25) (draw-thresh 0.35) (yawvel-thresh 0.05) (yawangle-thresh 0) ;; 卒論のため yawangle-thresh は 0
         (move-targets nil) (link-lists nil)  target-pos av-list lfoot-vec rfoot-vec lcount rcount prev-idx fan draw touch thresh
         (bvhfile (concatenate string "bvhfiles/" fname ".bvh") ) )
    (warn "vel-thresh ~a  z-thresh ~a~%point-angle-thresh ~a  point-z-thresh ~a  yawvel-thresh ~a  yawangle-thresh ~a~%touch-thresh ~a  draw-thresh ~a~% floor ~a ~%"
          vel-thresh z-thresh point-angle-thresh point-z-thresh yawvel-thresh yawangle-thresh touch-thresh draw-thresh floor)

    (setq *motionname* fname)

    (setq *m* (load-mcd bvhfile :scale 1
                        :bvh-robot-model-class kinect-sdk-sub-bvh-robot-model))

    (setq animation (send *m* :animation))
    (setq av-list (smoothfilter (send *m* :animation) :mod-list '(1 1 1 1 1)));; av-listの3から6要素はとってはならない

    (init-model-param);; 足面オフセットピッチ角度計算

    ;; リスト作成
    (setq lz-list-origin nil rz-list-origin nil lz-list nil rz-list nil lpos-list nil rpos-list nil)
    (setq lpitch-list-origin nil rpitch-list-origin nil lrot-list-origin nil rrot-list-origin nil)
    (setq diff-list nil diff-list-tmp nil)
    (dotimes (i (length animation))
      (push (elt (elt animation i) 14) lz-list-origin)
      (push (elt (elt animation i) 26) rz-list-origin)
      (push (subseq (elt animation i) 12 15) lpos-list)
      (push (subseq (elt animation i) 24 27) rpos-list)

      (setq lfoot-vec (v- (subseq (elt animation i) 15 18) (subseq (elt animation i) 12 15) )
            rfoot-vec (v- (subseq (elt animation i) 27 30) (subseq (elt animation i) 24 27) ) )
      (push (rad2deg (calc-pitch lfoot-vec)) lpitch-list-origin)
      (push (rad2deg (calc-pitch rfoot-vec)) rpitch-list-origin)
      (push (calc-rot lfoot-vec) lrot-list-origin)
      (push (calc-rot rfoot-vec) rrot-list-origin)
      )
    (nreverse lz-list-origin)(nreverse rz-list-origin) (nreverse lpos-list)(nreverse rpos-list)
    (nreverse lpitch-list-origin)(nreverse rpitch-list-origin) (nreverse lrot-list-origin)(nreverse rrot-list-origin)

    ;; zmp計算
    (setq zmp-list-origin (calc-zmp-from-skeleton-animation (smoothfilter animation :mod-list '(1 1 1 1 1 1 1) ) :old old))

    ;; 平滑化
    (setq lz-list (smoothfilter lz-list-origin :mod-list '(1 1 1))) (setq rz-list (smoothfilter rz-list-origin :mod-list '(1 1 1)))
    (setq lpos-list (smoothfilter lpos-list :mod-list '(1 1 1))) (setq rpos-list (smoothfilter rpos-list :mod-list '(1 1 1)))
    (setq lpitch-list (smoothfilter lpitch-list-origin) rpitch-list (smoothfilter rpitch-list-origin));; pitch
    (setq zmp-list (smoothfilter zmp-list-origin :mod-list '(1 1 1)));; zmp
    (setq lrot-list (smoothfilter lrot-list-origin) rrot-list (smoothfilter rrot-list-origin));; rot

    (setq lz-list-medi0 (copy-seq lz-list)  rz-list-medi0 (copy-seq rz-list))

    ;; 速度
    (setq lvel-list (smoothfilter (pos2vel lpos-list) :mod-list '(1 1 1 1 1)))
    (setq rvel-list (smoothfilter (pos2vel rpos-list) :mod-list '(1 1 1 1 1)))
    (setq lyawvel-list (smoothfilter (rot2yawvel lrot-list) :mod-list '(1 1 1 1 1)));; yaw vel
    (setq ryawvel-list (smoothfilter (rot2yawvel rrot-list) :mod-list '(1 1 1 1 1)))


    (setq l-list nil r-list nil)
    ;; 閾値走査
    (setq thresh (eval low))
    (while (<= thresh high)
      (warn "thresh ~a" thresh)
      (cond
       ( (= state *ground*) (setq z-thresh (eval thresh)       fan nil touch nil draw nil vel-thresh 100 point-angle-thresh 100 point-z-thresh (eval z-thresh)) )
       ( (= state *slide*)  (setq vel-thresh (eval thresh)     fan nil touch nil draw nil point-angle-thresh 100 point-z-thresh (eval z-thresh)) )
       ( (= state *point*)  (setq point-angle-thresh (eval thresh) fan nil touch nil draw nil ) )
       ( (= state *draw*)   (setq draw-thresh (eval thresh)    fan nil touch nil draw t) )
       ( (= state *touch*)  (setq touch-thresh (eval thresh)   fan nil touch t   draw nil) )
       ( (= state *fan*)    (setq yawvel-thresh (eval thresh)  fan t   touch nil draw nil yawangle-thresh 0) )
       )

      (setq lz-list (copy-seq lz-list-medi0) rz-list (copy-seq rz-list-medi0) )

      ;; ファン識別
      (when fan
        (setq lz-list (recognize-fan lz-list lyawvel-list :yawvel-thresh yawvel-thresh :yawangle-thresh yawangle-thresh :z-thresh z-thresh)
              rz-list (recognize-fan rz-list ryawvel-list :yawvel-thresh yawvel-thresh :yawangle-thresh yawangle-thresh :z-thresh z-thresh) ) )

      ;; 接地状態判定
      (setq lz-list (check-foot-state lz-list lvel-list lpitch-list
                                      :z-thresh z-thresh :vel-thresh vel-thresh :point-z-thresh point-z-thresh :point-angle-thresh point-angle-thresh)
            rz-list (check-foot-state rz-list rvel-list rpitch-list
                                      :z-thresh z-thresh :vel-thresh vel-thresh :point-z-thresh point-z-thresh :point-angle-thresh point-angle-thresh) )

      (setq lz-list-tmp (copy-seq lz-list) rz-list-tmp (copy-seq rz-list))

      ;; 前後フィルタ
      (when term
        (setq lz-list (min-term lz-list lz-list-medi0 :term term)) (setq rz-list (min-term rz-list rz-list-medi0 :term term)) )

      ;; タッチ識別
      (when touch
        (setq lz-list (separate-touch-ground lz-list rz-list lpos-list rpos-list zmp-list :thresh touch-thresh)
              rz-list (separate-touch-ground rz-list lz-list rpos-list lpos-list zmp-list :thresh touch-thresh) ) )

      ;; ドロー識別
      (when draw
          (setq lz-list (separate-draw-slide lz-list rz-list lpos-list rpos-list zmp-list :thresh draw-thresh)
                rz-list (separate-draw-slide rz-list lz-list rpos-list lpos-list zmp-list :thresh draw-thresh) ) )

      ;; 状態カウント
      (setq lcount 0 rcount 0)
      (dotimes (idx (length lz-list))
        (setq prev-idx (max 0 (1- idx)))
        (when (and (= (elt lz-list idx) state) (not (= (elt lz-list prev-idx) (elt lz-list idx))) )
          (incf lcount))
        (when (and (= (elt rz-list idx) state) (not (= (elt rz-list prev-idx) (elt rz-list idx))) )
          (incf rcount))
        )

      (warn "  l ~a  r ~a~%" lcount rcount)
      (push-back lcount l-list)(push-back rcount r-list)

      (setq thresh (+ thresh step))
      );; end while

    nil
    ))
