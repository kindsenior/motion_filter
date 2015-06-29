;; perfumeのbvhに合わせてhrp2が動く
#!/usr/bin/env roseus
#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(ros::roseus "hoge")
(load "~/prog/euslib/rbrain/eusbvh.l")
(load "package://roseus_tutorials/src/kinect-model-viewer.l")
(load "~/prog/euslib/demo/k-kojima/bvh/kinect-sdk-model-viewer.l")

(defclass perfume-bvh-robot-model
  :super bvh-robot-model
  :slots ())
(defmethod perfume-bvh-robot-model
  (:init (&rest args)
   (send-super* :init :coords (make-coords :rpy (float-vector pi/2 0 pi/2)) args)
   (setq larm (list
	       (find 'leftcollar    links :key #'(lambda (x) (send x :name)))
	       (find 'leftshoulder  links :key #'(lambda (x) (send x :name)))
	       (find 'leftelbow     links :key #'(lambda (x) (send x :name)))
	       (find 'leftwrist     links :key #'(lambda (x) (send x :name))))
	 rarm (list
	       (find 'rightcollar   links :key #'(lambda (x) (send x :name)))
	       (find 'rightshoulder links :key #'(lambda (x) (send x :name)))
	       (find 'rightelbow    links :key #'(lambda (x) (send x :name)))
	       (find 'rightwrist    links :key #'(lambda (x) (send x :name))))
	 lleg (list
	       (find 'lefthip       links :key #'(lambda (x) (send x :name)))
	       (find 'leftknee      links :key #'(lambda (x) (send x :name)))
	       (find 'leftankle     links :key #'(lambda (x) (send x :name)))
	       (find 'lefttoe       links :key #'(lambda (x) (send x :name))))
	 rleg (list
	       (find 'righthip      links :key #'(lambda (x) (send x :name)))
	       (find 'rightknee     links :key #'(lambda (x) (send x :name)))
	       (find 'rightankle    links :key #'(lambda (x) (send x :name)))
	       (find 'righttoe      links :key #'(lambda (x) (send x :name))))
	 torso (list
		(find 'chest        links :key #'(lambda (x) (send x :name)))
		(find 'chest2       links :key #'(lambda (x) (send x :name)))
		(find 'chest3       links :key #'(lambda (x) (send x :name)))
		(find 'chest4       links :key #'(lambda (x) (send x :name))))
	 head  (list
		(find 'neck        links :key #'(lambda (x) (send x :name)))
		(find 'head        links :key #'(lambda (x) (send x :name)))))
   ;;
   (send self :init-end-coords)
   (send self :init-root-link)
   ;;;
   self) ;; init
  ;;
  (:copy-state-to
   (robot &rest args)
   (send-super* :copy-state-to robot
		:conds '(((:larm :rarm) . (nil shoulder elbow nil))
			 ((:lleg :rleg) . (hip knee nil))
			 ((:head) . (neck)))
		args)
;   (send robot :rotate -pi/2 :x)
;   (send robot :rotate  pi/2 :z)
   )
  (:bvh-offset-rotate
   (name)
   (let ((r (unit-matrix 3)));;unit-matrix 単位行列
     (case name
       ((/left_shoulder)
	(setq r (rotate-matrix r (deg2rad -90) :z)))
       ((/right_shoulder)
	(setq r (rotate-matrix r (deg2rad 90) :z)))
       )
     r))
  (:fix-joint-order (jo limb)
   (case limb
     ((:larm :rarm)
      (mapc #'(lambda (x) (replace-elm x :y :x)) jo);;第2引数のリストに対して第1引数の関数を繰り返す
      (mapc #'(lambda (x) (replace-elm x :p :y)) jo);;mapcarとは新しい引数を返さない点が異なる
      (mapc #'(lambda (x) (replace-elm x :r :z)) jo))
     (t
      (mapc #'(lambda (x) (replace-elm x :y :y)) jo)
      (mapc #'(lambda (x) (replace-elm x :p :x)) jo) ;; cub
      (mapc #'(lambda (x) (replace-elm x :r :z)) jo))) ;; cyl
   jo)
  (:fix-joint-angle
   (i limb joint-name joint-order a)
   ;;hrpは同じ間接角を与える左右で同じ動きをするようになっている（正しいDH記法ではない）
   ;;それに対しbvhは正しいDH記法
   (when (and (memq limb '(:rarm))
	      (memq (elt joint-order i) '(:y)));;memq 第1引数が第2引数の要素かどうか調べる
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:larm))
	      (memq (elt joint-order i) '(:z)))
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:rleg))
	      (memq (elt joint-order i) '( :y :z)))
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
  ) ;; perfume-bvh-robot-model

;; (require :hrp4c "package://hrpsys_ros_bridge/models/hrp4c.l")
;; (if (not (boundp '*robot*))
;;     (setq *robot* (hrp4)))

(if (not (boundp '*hrp2*))
    (setq *robot* (hrp2jsk-simple)))

(defun start ()
  ;; (setq *m* (load-mcd "bvhfiles/nocchi.bvh" :scale 10 :bvh-robot-model-class perfume-bvh-robot-model))
  (setq *m* (load-mcd "bvhfiles/test.bvh" :scale 1 :bvh-robot-model-class kinect-sdk-bvh-robot-model))
  ;;*m* motion-capture-data
  (objects (list *robot* (send *m* :model) (make-cube 100 100 100)));; *m* . model が棒人間（bvhのロボットモデル）
  (catch :animate
    (dotimes (i (send *m* :frame-length))
      ;; (print i)
      (send *m* :frame i);;captureデータのフレーム番号をセット
      ;; (send *m* :model :copy-state-to *robot*);;*m* . model が perfume-robot-model クラス
      (send (get *viewer* :pickviewer) :look-all)
      (if (select-stream (list *standard-input*) 1.0e-8) (throw :animate nil));;catch内が nil を返して終了する
      (unix::usleep (* 1000 33))
      (x::window-main-one))
    )
  )

;; (start)