;; bvhに合わせてhrp2がopenhrp上で動く
;; roslaunchしてから実行
#!/usr/bin/env/ roseus
#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(load "package://jsk_hrpsys_ros_bridge/scripts/hrp2jsk-interface.l")
(load "package://jsk_hrpsys_ros_bridge/scripts/hrp4r-interface.l")

(load "euslib/demo/k-kojima/bvh/perfume-bvh-robot-model.l")
(load "euslib/demo/k-kojima/myeuslib.l")
(load "euslib/irteus_proposals/motion-lib-proposal.l")

(ros::roseus "hoge")

(if (not (boundp '*hrp2*))
    (setq *robot* (hrp2jsk-simple)))
(if (not (boundp '*ri*))
    (setq *ri* (instance hrp2jsk-interface :init)))
;; (if (not (boundp '*hrp4r*))
;;     (setq *robot* (hrp4r)))
;; (if (not (boundp '*ri*))
;;     (setq *ri* (instance hrp4r-interface :init)))

;; (pickview)
;; (objects (list *hrp2* ))

(defun make-sequence ()
  ;; (ros::rate 1);1Hz

  (setq bvhfile (str+  "/home/" (unix::getenv "USER") "/prog/euslib/demo/k-kojima/bvh/bvhfiles/nocchi.bvh") );;bvhファイル読み込み

  ;; (setq *m* (load-mcd "bvhfiles/nocchi.bvh" :scale 10
  ;;                     :bvh-robot-model-class perfume-bvh-robot-model))

  (setq *m* (load-mcd bvhfile :scale 10
                      :bvh-robot-model-class perfume-bvh-robot-model));;bvhモデル作成
  (send *ri* :set-interpolation-mode :linear);;線形補完
  ;; (send *robot* :legs :move-end-pos #f(0 0 0) :world)
  ;;(send *robot* :move-centroid-on-foot :both '(:rleg :lleg))
  (setq av-list nil tm-list nil)

  (catch :animate
    (dotimes (i (send *m* :frame-length))

      (warn "bvh frame ~A~%" i)
      (send *m* :frame i)
      (send *m* :model :copy-state-to *robot*)

;;      (send *ri* :angle-vector (send *robot* :hrp-angle-vector) 25);;msec bvhfile:0.025sec?
;;      (send *ri* :wait-interpolation)

      (push (send *robot* :hrp-angle-vector) av-list);;angle-vector列作成
      (push 25 tm-list)

      ;; (send *ri* :state :reference-vector);;今はnil
      ;; (print (send *ri* :state :potentio-vector))
      ;; (print (send *ri* :state :torque-vector))

      ;; (send *ri* :state)
      ;; (ros::spin-once)
      ;; (ros::sleep);;1Hzに合わせるためにsleep?
      ;; (print (assoc :velocity (*ri* . robot-state)))

      ;; (send *pickview* :draw-objects)

      ;; (send (get *viewer* :pickviewer) :look-all)
      (if (select-stream (list *standard-input*) 1.0e-8) (throw :animate nil))
      ;; (x::window-main-one)
      )
    )
  )

(defun dance ()
  ;; play angle-vector-sequence
  (send *ri* :angle-vector-sequence (reverse av-list) (reverse tm-list))
)

;; (make-sequence)
;; (dance)

;; 2.irteusgl$ hrp2jsk-init
;; 4.irteusgl$ send *hrp2jsk* :larm :wrist-p :joint-angle 20

;; (send *robot* :larm :shoulder-p :joint-angle 60)
;; (send *ri* :angle-vector (send *robot* :hrp-angle-vector) 25)