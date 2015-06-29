;; sdkのbvhに合わせて棒人間モデルが動く

(load "euslib/rbrain/eusbvh.l")
(load "euslib/demo/k-kojima/bvh/kinect-sdk-sub-model-viewer.l")
;; (load "euslib/demo/k-kojima/bvh/filter.l")
(load "euslib/demo/k-kojima/bvh/frame-class.l")

(load (concatenate string "~/ros/" (unix:getenv "ROS_DISTRO") "/src/rtm-ros-robotics/rtmros_tutorials/hrpsys_ros_bridge_tutorials/models/hrp2jsk.l"))

(warn "This is the program to play bvhfile of kinect sdk bvh model.~%usage: (play bvhfile-name)~%Push Enter to stop playing.~%")

(if (not (boundp '*robot*)) (setq *robot* (HRP2JSK)))

(setq stdout nil *jsk-debug* t)
(defun play (fname)
  (let ( )
    (setq *m* (load-mcd fname :scale 1 :bvh-robot-model-class kinect-sdk-sub-bvh-robot-model))

    (setq animation (send *m* :animation))
    ;; (setq av-list (smoothfilter (send *m* :animation) :mod-list '(1 1 1 1 1)))

    ;; (objects (list  (send *m* :model) *robot*))
    (objects (list  (send *m* :model)))

    (catch :animate
      (dotimes (i (send *m* :frame-length))
        (send *m* :frame i);; 全体位置・姿勢セット

        ;; 棒人間モデルの姿勢設定
        (send *m* :model :set-joint-angle (elt animation i))

        ;; copy-state-toができない
        ;; (send (send *m* :model) :copy-state-to *robot*)

        (send *viewer* :draw-objects)
        (unix::usleep (* 1000 33))
        (if (select-stream (list *standard-input*) 1.0e-8) (throw :animate nil))
        (x::window-main-one)
        
        );; end dotimes
      );; end animate

    nil
    ))

    


