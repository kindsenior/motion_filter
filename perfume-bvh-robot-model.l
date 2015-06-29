;; perfurmeのbvhからeusモデルを生成する

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
		:conds '(((:larm :rarm) . (nil shoulder elbow wrist))
			 ((:lleg :rleg) . (hip knee ankle))
			 ((:head) . (neck)))
		args)
;   (send robot :rotate -pi/2 :x)
;   (send robot :rotate  pi/2 :z)
   )
  (:bvh-offset-rotate
   ;; :angle-vectorで全部0を入れたときに気を付けになるように調整？？
   (name)
   (let ((r (unit-matrix 3)));;unit-matrix 単位行列
     (case name
       ((leftshoulder)
        (setq r (rotate-matrix r (deg2rad -90) :z)))
       ((rightshoulder)
        (setq r (rotate-matrix r (deg2rad 90) :z)))
       )
     r))
  (:fix-joint-order
   ;; 各関節のrpyを気を付けの姿勢のxyzに変換
   (jo limb)
   (case limb
     ((:larm :rarm)
      (mapc #'(lambda (x) (replace-elm x :y :x)) jo)
      (mapc #'(lambda (x) (replace-elm x :p :y)) jo)
      (mapc #'(lambda (x) (replace-elm x :r :z)) jo))
     (t
      (mapc #'(lambda (x) (replace-elm x :y :y)) jo)
      (mapc #'(lambda (x) (replace-elm x :p :x)) jo) ;; cub
      (mapc #'(lambda (x) (replace-elm x :r :z)) jo))) ;; cyl
   jo)
  (:fix-joint-angle
   (i limb joint-name joint-order a)
   ;; 気を付けの姿勢での各関節のxyz軸の回転方向を設定
   ;;hrp2は同じ間接角を与えると左右で同じ動きをするようになっている（正しいDH記法ではない）
   ;;それに対しbvhは正しいDH記法
   (when (and (memq limb '(:rarm))
	      (memq (elt joint-order i) '(:y)));;memq 第1引数が第2引数の要素かどうか調べる
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:larm))
	      (memq (elt joint-order i) '(:x)))
     (setf (elt a i) (- (elt a i)))
     )
   (when (and (memq limb '(:rleg))
	      (memq (elt joint-order i) '()))
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
