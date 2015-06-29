(load "euslib/demo/k-kojima/bvh/filter.l")

#-:ros
(unless (find-package "ROS") (make-package "ROS"))

(defclass kinect-sdk-sub-bvh-robot-model
  :super bvh-robot-model
  :slots ())
(defmethod kinect-sdk-sub-bvh-robot-model
  (:init (&rest args)
   (send-super* :init args)
   (setq rarm (list
               (find '/left_shoulder  links :key #'(lambda (x) (send x :name)))
               (find '/left_elbow     links :key #'(lambda (x) (send x :name)))
               (find '/left_wrist     links :key #'(lambda (x) (send x :name))))
         larm (list
               (find '/right_shoulder links :key #'(lambda (x) (send x :name)))
               (find '/right_elbow    links :key #'(lambda (x) (send x :name)))
               (find '/right_wrist    links :key #'(lambda (x) (send x :name))))
         rleg (list
               (find '/left_hip       links :key #'(lambda (x) (send x :name)))
               (find '/left_knee      links :key #'(lambda (x) (send x :name)))
               (find '/left_ankle     links :key #'(lambda (x) (send x :name))))
         lleg (list
               (find '/right_hip      links :key #'(lambda (x) (send x :name)))
               (find '/right_knee     links :key #'(lambda (x) (send x :name)))
               (find '/right_ankle    links :key #'(lambda (x) (send x :name))))
         torso (list
                (find '/torso         links :key #'(lambda (x) (send x :name))))
         head  (list
                (find '/neck          links :key #'(lambda (x) (send x :name)))))
   ;;
   (mapcar #'(lambda (limb name)
	       (eval `(let ((l (find-if #'(lambda (x) (eq (send x :name) 'site)) (send (car (last ,limb)) :descendants))))
                  ;; (send l :name ',name)
                  (if l (send  l :name ',name))
                  (setq ,limb (append ,limb (list l)))))
         )
	   (list 'rarm 'larm 'rleg 'lleg 'head)
	   (list '/right_hand '/left_hand '/right_foot '/left_foot '/head))
   ;;
   (send self :init-end-coords)
   (send self :init-root-link)

   ;;
   ;;;
   self) ;; init
  (:init-end-coords
   ()
   (let (parent-link)
     ;;
     (setq parent-link (find '/left_hand links :key #'(lambda (x) (send x :name))))
     (setq rarm-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send rarm-end-coords :rotate -pi/2 :x)
     (send rarm-end-coords :rotate  pi   :z)
     (send parent-link :assoc rarm-end-coords)
     ;;
     (setq parent-link (find '/right_hand links :key #'(lambda (x) (send x :name))))
     (setq larm-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send larm-end-coords :rotate  pi   :x)
     (send parent-link :assoc larm-end-coords)
     ;;
     (setq parent-link (find '/left_foot links :key #'(lambda (x) (send x :name))))
     (setq rleg-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send rleg-end-coords :rotate -pi/2 :x)
     (send rleg-end-coords :rotate  pi/2 :z)
     (send parent-link :assoc rleg-end-coords)
     ;;
     (setq parent-link (find '/right_foot links :key #'(lambda (x) (send x :name))))
     (setq lleg-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send lleg-end-coords :rotate -pi/2 :x)
     (send lleg-end-coords :rotate  pi/2 :z)
     (send parent-link :assoc lleg-end-coords)
     ;;
     (setq parent-link (find '/torso links :key #'(lambda (x) (send x :name))))
     (setq torso-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send torso-end-coords :rotate  pi/2 :y)
     (send torso-end-coords :rotate -pi/2 :x)
     (send parent-link :assoc torso-end-coords)
     ;;
     (setq parent-link (find '/head links :key #'(lambda (x) (send x :name))))
     (setq head-end-coords (make-cascoords :coords (send parent-link :copy-worldcoords)))
     (send head-end-coords :rotate  pi :y)
     (send parent-link :assoc head-end-coords)
     ))
  ) ;; kinect-sdk-sub-bvh-robot-model

;; (defun make-kinect-sdk-sub-bvh-robot-model ()
;;   (instance kinect-sdk-sub-bvh-robot-model :init :tree
;;             '((root /torso)
;;               (offset #f(0.0 0.0 0.0))
;;               (channels 6 xposition yposition zposition zrotation xrotation yrotation)
;;               ((joint /left_hip)
;;                (offset #f(90 -200 0))
;;                (channels 3 zrotation xrotation yrotation)
;;                ((joint /left_knee)
;;                 (offset #f(0 -400 0))
;;                 (channels 3 zrotation xrotation yrotation)
;;                 ((end /left_foot)
;;                  (offset #f(0 -380 0)))))
;;               ((joint /right_hip)
;;                (offset #f(-90 -200 0))
;;                (channels 3 zrotation xrotation yrotation)
;;                ((joint /right_knee)
;;                 (offset #f(0 -400 0))
;;                 (channels 3 zrotation xrotation yrotation)
;;                 ((end /right_foot)
;;                  (offset #f(0 -380 0)))))
;;               ((joint /left_shoulder)
;;                (offset #f(140 190 0))
;;                (channels 3 zrotation xrotation yrotation)
;;                ((joint /left_elbow)
;;                 (offset #f(280 0 0))
;;                 (channels 3 zrotation xrotation yrotation)
;;                 ((end /left_hand)
;;                  (offset #f(260 0 0)))))
;;               ((joint /right_shoulder)
;;                (offset #f(-140 190 0))
;;                (channels 3 zrotation xrotation yrotation)
;;                ((joint /right_elbow)
;;                 (offset #f(-280 0 0))
;;                 (channels 3 zrotation xrotation yrotation)
;;                 ((end /right_hand)
;;                  (offset #f(-260 0 0)))))
;;               ((joint /neck)
;;                (offset #f(0 200 0))
;;                (channels 3 zrotation xrotation yrotation)
;;                ((end /head)
;;                 (offset #f(0 200 0)))))
;;             )) ;; make-kinect-sdk-sub-bvh-robot-model

;;
;; Kinect
;;
;; reading data from kinect

(defmethod kinect-sdk-sub-bvh-robot-model
  (:copy-state-to
   (robot &rest args)
   (send-super* :copy-state-to robot
		:conds '(((:larm :rarm) . (nil shoulder elbow wrist))
			 ((:lleg :rleg) . (hip knee ankle ))
			 ((:head) . (neck)))
		args)
   ;; (send robot :rotate -pi/2 :x)
   ;; (send robot :rotate -pi/2 :z)
   )
  (:bvh-offset-rotate
   (name)
   (let ((r (unit-matrix 3)))
     (case name
       ;; ((/left_shoulder)
       ;;  (setq r (rotate-matrix r (deg2rad 90) :x)))
       ;; ((/right_shoulder)
       ;;  (setq r (rotate-matrix r (deg2rad -90) :x)))
       ;; ((/left_hip)
       ;;  (setq r (rotate-matrix r (deg2rad 90) :y)))
       ;; ((/right_hip)
       ;;  (setq r (rotate-matrix r (deg2rad 90) :y)))
       ((/left_ankle)
        (print "hoge")
        (setq r (rotate-matrix r (deg2rad -50) :y)))
       ((/right_ankle)
        (setq r (rotate-matrix r (deg2rad -50) :y)))
       )
     r))
  (:fix-joint-order (jo limb)
   (case limb
     ((:larm :rarm)
      (mapc #'(lambda (x) (replace-elm x :y :z)) jo)
      (mapc #'(lambda (x) (replace-elm x :p :y)) jo)
      (mapc #'(lambda (x) (replace-elm x :r :x)) jo))
     (t
      (mapc #'(lambda (x) (replace-elm x :y :z)) jo)
      (mapc #'(lambda (x) (replace-elm x :p :y)) jo) ;; cub
      (mapc #'(lambda (x) (replace-elm x :r :x)) jo))) ;; cyl
   jo)
  (:fix-joint-angle
   (i limb joint-name joint-order a)
   (when (and (memq limb '(:rarm))
	      ;; (memq (elt joint-order i) '(:x :y :z)));; for hrp4
	      (memq (elt joint-order i) '(:x  :z)));; for hrp2
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:larm))
	      (memq (elt joint-order i) '()))
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:rleg))
	      ;; (memq (elt joint-order i) '(:x :z)));; for hrp4
	      (memq (elt joint-order i) '(:x :z)));; for hrp2
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:lleg))
	      (memq (elt joint-order i) '()))
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:head))
	      (memq (elt joint-order i) '(:x :y)))
     (setf (elt a i) (- (elt a i)))
     )

   a)
  ) ;; kinect-sdk-sub-bvh-robot-model

#+:ros
(defmethod kinect-sdk-sub-bvh-robot-model
  (:copy-state-from-tf
   (tf-listener)
   (let (from to m (tm (ros::time)) (ret t) (last-time)) 
     (dotimes (i 4)
       (setq last-time (send tf-listener :get-latest-common-time "map" (format nil "/torso_~d" i)))
       (if last-time (setq last-time (send (ros::time- (ros::time-now) last-time) :to-sec)))
       (when (and last-time (< last-time 1))
         (dolist (j (cdr (send self :joint-list)))
           (setq from (format nil "~a_~d" (string-downcase (send (send j :parent-link) :name)) i)
                 to   (format nil "~a_~d" (string-downcase (send j :name)) i))
           (setq m (send tf-listener :lookup-transform from to tm))
           (if m
               (send j :joint-angle (map cons #'rad2deg (matrix-log (send m :worldrot))))
             (setq ret nil)))
         (setq m (send tf-listener :lookup-transform "/map" (format nil "/torso_~d" i) tm))
         (if m
             (send (car (send self :joint-list))
                   :joint-angle (coerce (append (coerce (send m :worldpos) cons)
                                                (map cons #'rad2deg (matrix-log (send m :worldrot)))) float-vector))
           (setq ret nil))
         (return-from :copy-state-from-tf ret)))
     nil)) )

;; animationのpositionに合うように関節角を計算
(defmethod kinect-sdk-sub-bvh-robot-model
  ;; 脚関節角度計算
  (:calc-leg-joint-angle
   (leg-pos-list coord limb type)
   (let (q0 q1 q2 q3 q4 y yul yfu yfl u l f a_ n regression-plane (q3-thresh 10) rate knee-axis-sign)
     ;; 体中心座標系に変更
     (dotimes (i (length leg-pos-list)) (setf (elt leg-pos-list i) (send coord :inverse-rotate-vector (elt leg-pos-list i))))

     ;; (setq leg-pos-list (project-to-regression-plane leg-pos-list (car leg-pos-list)));; 回帰平面に投影
     (setq n (car (lmeds leg-pos-list)));; 法線ベクトル 主成分回帰
     (setq regression-plane (make-plane :normal n :point (car leg-pos-list)));; 回帰平面
     (setq leg-pos-list (mapcar #'(lambda (leg-pos) (send regression-plane :foot leg-pos)) leg-pos-list))

     ;; ;; 回帰後の関節座標表示
     ;; (dotimes (j 4)
     ;;   (if (equal limb :lleg)
     ;;       (send (elt regression-obj-list j) :locate (send coord :rotate-vector (elt leg-pos-list j)) :world)
     ;;     (send (elt regression-obj-list (+ j 4)) :locate (send coord :rotate-vector (elt leg-pos-list j)) :world) )
     ;;   )

     (setq u (normalize-vector (v- (elt leg-pos-list 1) (elt leg-pos-list 0)))
           l (normalize-vector (v- (elt leg-pos-list 2) (elt leg-pos-list 1)))
           f (normalize-vector (v- (elt leg-pos-list 3) (elt leg-pos-list 2))) )

     (setq yul (normalize-vector (v* u l))
           yfu (normalize-vector (v* f u));; 最後の関節は負方向しか回らないので逆順の外積
           yfl (normalize-vector (v* f l)));; yfは伸膝時しか使わないからlよりもuの方が精度がいいはず

     ;; q3,q4
     (setq q3 (acos (v. u l)))

     ;; 膝関節の回転方向
     ;; (if (or (= (abs (v. yfl yul)) 0) (> (rad2deg q3) 10));; 10度付近での処理は要検討
     (if (= (abs (v. yfl yul)) 0)
         (setq knee-axis-sign 1)
       (setq knee-axis-sign (/ (v. yfl yul) (abs (v. yfl yul)))) )

     (setq yul (scale  knee-axis-sign yul));; 膝が逆関節の時はyulを逆向きにする

     (setq q3 (* knee-axis-sign q3)
           q4 (- (deg2rad 50) (acos (v. l f))) )


     ;; y設定
     ;; (setq q3-thresh 10.0)
     ;; ;; (cond;; y単純切り替え方式
     ;; ;;  ( (eps= q3 (deg2rad 0) (deg2rad 10))
     ;; ;;    (format stdout  "knee 0~%")
     ;; ;;    (setq y yf))
     ;; ;;  ( t (setq y yu)))
     ;; ;; (setq y (v+ (scale (sin q3) yu) (scale (- 1 (abs (sin q3))) yf)));; yマージ方式
     ;; (cond;; 伸膝時yマージ方式
     ;;  ( (eps= q3 (deg2rad 0) (/ (deg2rad q3-thresh) 1));; 伸膝時
     ;;    (format stdout  "knee 0~%")
     ;;    (setq y yfu) )
     ;;  ;; ( (eps= q3 (deg2rad 0) (deg2rad q3-thresh));; 亜伸膝時
     ;;  ;;   (format stdout  "knee near 0~%")
     ;;  ;;   (setq rate (- (* 2 (/ q3 (deg2rad q3-thresh) )) 1) )
     ;;  ;;   (setq y (v+ (scale (- 1 rate) yfu) (scale rate yul))) )
     ;;  ( t (setq y  yul)));; 膝関節が直角の時はabort     

     (cond
      ( (equal type :yfl) (setq y yfl) )
      ( (equal type :yonekura)
        (cond;; y単純切り替え方式
         ( (eps= q3 (deg2rad 0) (deg2rad q3-thresh))
           (setq y yfl) )
         ( t (setq y yul) ) )
        )
      ( t 
        ;; 回帰平面法線ベクトル方式
        (setq y yfl)
        ;; (setq n (calc-regression-plane leg-pos-list));; 回帰平面法線ベクトル計算
        (cond ( (< (v. y n) 0) (setq y (scale -1 n)) ) (t (setq y n) ) ) )
      )

     ;; q1
     (setq q1 (asin (elt y 2)))

     ;; q0,q2
     (cond
      ( (eps= (sin q1) 1.0 1.0e-6);; この0.1は要検討
        (format stdout "sin q1 == 1~%")
        (setq q0 0;; 本当はprefq0にすべき？？
              q2 (* -1 (atan (elt u 0) (* -1 (elt u 1))) ) ))
      ( (eps= (sin q1) -1.0 1.0e-6)
        (format stdout "sin q1 == -1~%")
        (setq q0 0
              q2 (* -1 (atan (elt u 0) (elt u 1)) ) ));; absを使えば場合分けなし？
      ( t
        ;; (setq a_ (/ (elt y 0) (cos q1) -1.0))
        ;; (cond ( (> a_ 1.0) (setq a_ 1.0) )
        ;;       ( (< a_ -1.0) (setq a_ -1.0) ))
        ;; (setq q0 (asin a_));; atanとか使うべき？？
        (setq q0 (atan (/ (elt y 0) (cos q1) -1.0) (/ (elt y 1) (cos q1)) ))
        (cond
         ( (eps= (sin q1) 0.0 1.0e-6)
           (format stdout "sin q1 == 0~%")
           (cond ( (eps= (cos q0) 0.0 1.0e-6) (setq q2 (asin (/ (elt u 1) -1.0 (sin q0)))) 
                   (format stdout "cos q0 == 0"))
                 ( t (setq q2 (asin (/ (elt u 0) -1.0 (cos q0))))) ) )
         ( t
           (format stdout "     default~%")
           (setq q2 (asin (/ (+ (* (cos q0) (sin q1) (elt u 0))
                                (* (sin q0) (sin q1) (elt u 1)))
                             (sin q1) -1.0) ) )
           ;; (setq q2 (atan (* (+ (* (cos q0) (elt u 0)) (* (sin q0) (elt u 1)))
           ;;                   -1.0)
           ;;                (abs (/ (+ (* (sin q0) (elt u 0)) (* -1 (cos q0) (elt u 1)))
           ;;                   -1.0 (sin q1) )) ) )
           )
         )))

     ;; (format stdout "~a~%"
     ;;         (float-vector (rad2deg q2) (rad2deg q1) (rad2deg q0) 0 (rad2deg q3) 0 0 (rad2deg q4) 0))
     (float-vector (rad2deg q0) (rad2deg q1) (rad2deg q2);; z x y(bvhの定義順)
                   0 0 (rad2deg q3)
                   0 0 (rad2deg q4))
     ))

  ;; 腕関節角度計算
  (:calc-arm-joint-angle
   (arm-pos-list coord limb)
   (let (q0 q1 q2 q3 q4 q5 q6 y yul u u_ l h h_ a_ (q3-thresh (deg2rad 10)) rate elbow-axis-sign z_  R-yxz-y )
     ;; 体中心座標系に変更
     (dotimes (i (length arm-pos-list)) (setf (elt arm-pos-list i) (send coord :inverse-rotate-vector (elt arm-pos-list i))))

     ;; (setq arm-pos-list (project-to-regression-plane arm-pos-list (car arm-pos-list)));; 回帰平面に投影

     ;; 左右対応
     (cond
      ( (equal limb :larm) (setq z_ #f(0 0 -1) ) )
      ( (equal limb :rarm) (setq z_ #f(0 0  1) ) )
      ( t (warning-message 1 "limb is void~%") )
      )

     (setq u (normalize-vector (v- (elt arm-pos-list 1) (elt arm-pos-list 0)))
           l (normalize-vector (v- (elt arm-pos-list 2) (elt arm-pos-list 1)))
           h (normalize-vector (v- (elt arm-pos-list 3) (elt arm-pos-list 2))) )

     (setq yul (normalize-vector (v* u l)))

     ;; (setq elbow-axis-sign (/ (v. yfl yul) (abs (v. yfl yul))));; 肘関節の回転方向
     (setq elbow-axis-sign 1)
     ;; (setq yul (scale  elbow-axis-sign yu));; 膝が逆関節の時はyulを逆向きにする

     ;; q3
     (setq q3 (* elbow-axis-sign (* -1 (acos (v. u l)))))

     ;; q0
     ;; q1が90度近くの時のことを考慮しなくていい??
     (setq q0 (atan (* -1 (elt u 0)) (* -1 (elt u 2))))

     ;; q1
     (setq u_ (transform (inverse-matrix (rotation-matrix q0 :y)) u))
     (setq q1 (atan (elt u_ 1) (* -1 (elt u_ 2))))

     ;; q2
     (cond
      ( (eps= (sin q1) 0.0 1.0e-6)
        (if (eps= (cos q0) 0.0 1.0e-6) (setq q2 (asin (/ (elt yul 2) (sin q0) -1))) (setq q2 (asin (/ (elt yul 0) (cos q0))))) )
      ( t (setq q2 (asin (- (* (cos q0) (elt yul 0)) (* (sin q0) (elt yul 2))))) )
      )

     ;; 肘が伸びきっている場合
     (when (< (abs q3) q3-thresh)
       (setq rate (/ (abs q3) q3-thresh))
       (setq q2 (* rate q2))
       )

     ;; q6
     (setq q6 0)

     ;; 以下要改良 wrist-yawの動きが速すぎる
     ;; h_
     (setq R-yxz-y (reduce #'m* (list (rotation-matrix q0 :y) (rotation-matrix q1 :x) (rotation-matrix q2 :z) (rotation-matrix q3 :y))) )
     (setq h_ (transform (inverse-matrix R-yxz-y) h))


     (setq q4 (atan (elt h_ 0) (* -1 (elt h_ 1))))
     (cond
      ( (> q4 (deg2rad 90))  (setq q4 (- q4 (deg2rad 180))) )
      ( (< q4 (deg2rad -90)) (setq q4 (+ q4 (deg2rad 180))) )
      )
     
     ;; q5
     (cond
      ( (eps= (cos q4) 0 1.0e-6) (setq q5 (asin (/ (elt h_ 1) (cos q4)))) )
      ( t (setq q5 (asin (/ (elt h_ 0) (sin q4) -1))) )
      )

     ;; 手首が伸びきっている場合
     (when (< (abs q5) q3-thresh)
       (setq rate (/ (abs q5) q3-thresh))
       (setq q4 (* rate q4))
       )
     (setq q4 0 q5 0)

     (format stdout "~a~%"
             (float-vector (rad2deg q2) (rad2deg q1) (rad2deg q0) (rad2deg q3) 0 0 (rad2deg q4) (rad2deg q5) (rad2deg q6)))
     (float-vector (rad2deg q0) (rad2deg q1) (rad2deg q2);; y x z(bvhの定義順)
                   (rad2deg q3) 0 0
                   (rad2deg q4) (rad2deg q5) (rad2deg q6))
     ))

  ;; animationのpositionに合うように関節角をセット
  (:set-joint-angle
   (av &key (type nil))
   (let (lleg-pos-list rleg-pos-list  larm-pos-list rarm-pos-list waist-coord)

     (setq lleg-pos-list (list (subseq av 6 9) (subseq av 9 12)
                               (subseq av 12 15) (subseq av 15 18)));; 左足
     (setq rleg-pos-list (list (subseq av 18 21) (subseq av 21 24)
                               (subseq av 24 27) (subseq av 27 30)));; 右足
     (setq larm-pos-list (list (subseq av 30 33) (subseq av 33 36)
                               (subseq av 36 39) (subseq av 39 42)));; 左手
     (setq rarm-pos-list (list (subseq av 42 45) (subseq av 45 48)
                               (subseq av 48 51) (subseq av 51 54)));; 右手

     (setq waist-coord (make-coords :rpy (car (send (car (send self :torso)) :rpy-angle))))

     (send self :angle-vector
           (concatenate float-vector
                        (float-vector (elt av 0) (elt av 1) (elt av 2))
                        (subseq av 3 6)
                        (send self :calc-leg-joint-angle rleg-pos-list waist-coord :rleg type)
                        (send self :calc-leg-joint-angle lleg-pos-list waist-coord :lleg type)
                        (send self :calc-arm-joint-angle rarm-pos-list waist-coord :rarm)
                        (send self :calc-arm-joint-angle larm-pos-list waist-coord :larm)
                        #f(0 0 0)
                        ;; #f(20 -10 0  0 0 -60  70 0 20   -20 10 0  0 0 -60  -70 0 20   0 0 0)
                        ))
     ))

  )

#+:ros
(defun test-kinect (&key robot fname loop-hook) ;; :fname "test-kinect.bvh", loop-hook is a function to call inside do-until-key loop
  (let ((floor (make-cube 2000 1000 10 :pos #f(1000 0 0))) f)
    (if fname (setq f (open fname :direction :output)))
    (ros::roseus "kinect_bvh")
    (if (not (boundp '*tl*))
        (setq *tl* (instance ros::transform-listener :init)))
    (setq b (make-kinect-sdk-sub-bvh-robot-model))
    (objects (list floor b))
    (if robot (objects robot))
    ;;(defun find-node (name) (find name *arrows* :key #'(lambda (x) (send x :name)) :test #'string=))
    (if f (send b :dump-hierarchy f))
    (do-until-key
     (let ()
       (when (send b :copy-state-from-tf *tl*)
         (format t "kinect robot : larm pos=~A, rarm pos=~A (world)~%"
                 (send b :larm :end-coords :worldpos)
                 (send b :rarm :end-coords :worldpos))
         (format t "             : larm pos=~A, rarm pos=~A (local)~%"
                 (send (send (send b :torso :end-coords) :transformation (send b :larm :end-coords)) :worldpos)
                 (send (send (send b :torso :end-coords) :transformation (send b :rarm :end-coords)) :worldpos)))
       (when robot
         (send b :copy-state-to robot))
       (if (functionp loop-hook) (funcall loop-hook))
       (send (get *viewer* :pickviewer) :look-all)
       (x::window-main-one)
       (if f (send b :dump-motion f))
       )) ;; do-until-key
    ))

;; (test-kinect)



