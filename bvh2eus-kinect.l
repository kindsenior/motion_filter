;; hrp2がkinectで作成したbvhに合わせて動く
#!/usr/bin/env roseus
#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(load "~/prog/euslib/rbrain/eusbvh.l")
(load "package://roseus_tutorials/src/kinect-model-viewer.l")

(load "~/prog/euslib/demo/k-kojima/bvh/filter.l")

;; (if (not (boundp '*hrp2*))
;;     (setq *robot* (hrp2jsk-simple-detail)))

(if (not (boundp '*box*)) (setq *box* (make-cube 1000 1000 1)))
(send  *box* :locate #f(2250 0 0) :world)
(send *box* :set-color :blue)

(setq pos-offset #f(75 0 0))
(when (not (boundp '*model-lfoot*))
  (setq *model-lfoot* (make-cube 150 75 1))(send *model-lfoot* :locate pos-offset)
  (setq *model-lfoot* (instance bodyset-link :init (make-cascoords) :bodies (list *model-lfoot*)))
  (send *model-lfoot* :set-color :red))
(when (not (boundp '*model-rfoot*))
  (setq *model-rfoot* (make-cube 150 75 1))(send *model-rfoot* :locate pos-offset)
  (setq *model-rfoot* (instance bodyset-link :init (make-cascoords) :bodies (list *model-rfoot*))))


(setq start-frame 2000)
(setq rate 10)
(setq ankle-lang-list nil  ankle-rang-list nil)
(defun start ( &key (path "/home/kunio/Dropbox/choreonoid/") (fname "square-slide") )
  (let ( (bvhfile (concatenate string path "bvhfiles/" fname ".bvh"))
         (datfile (concatenate string path "ankle_angle/" fname ".dat"))  )
    (setq *m* (load-mcd bvhfile :scale 1 :bvh-robot-model-class kinect-bvh-robot-model))
    ;;*m* motion-capture-data
    ;; (objects (list *robot* (send *m* :model) *box*));; *m* . model が棒人間（bvhのロボットモデル）
    ;; (objects (list  (send *m* :model) *box*));; *m* . model が棒人間（bvhのロボットモデル）

    (setq av-list nil)
    (setq tm-list nil)

    ;; (unix:sleep 3)
    (catch :make-list
      (warn "now making list~%")
      (dotimes (i (send *m* :frame-length))

        ;; (when (and (not (equal (send (car (send *m* :model :torso)) :worldpos)  #f(0 0 0) )) (< (+ i start-frame) (send *m* :frame-length)) )
        (when (< (+ i start-frame) (send *m* :frame-length))
          (send *m* :frame (+ i start-frame));;captureデータのフレーム番号をセット

          ;; (send *m* :model :copy-state-to *robot*);;*m* . model が kinect-bvh-robot-model クラス
          ;; (send (get *viewer* :pickviewer) :look-all)
          (push (send *m* :model :angle-vector) av-list)
          (push 30 tm-list)

          ;; (send *pickview* :draw-objects)
          (if (select-stream (list *standard-input*) 1.0e-8) (throw :make-list nil))

          ;; (unix:usleep (* 1000 10))
          ;; (x::window-main-one))
          )))
    (nreverse av-list)(nreverse tm-list)
    (smoothfilter av-list :mod-list '(1 1 1 1 1))

    (warn "finish making list~%")

    (objects (list  (send *m* :model) *box* *model-lfoot* *model-rfoot* ));; *m* . model が棒人間（bvhのロボットモデル）


    ;; ankle angle list 作成
    (setq ankle-lang-list nil  ankle-rang-list nil)
    (with-open-file
     (df datfile :direction :input)
     (let ( (eof (gensym)) line ang-list)
       (while (not (eq eof (setq line (read-line df nil eof))))
         ;; (setq line (eval (read-from-string line)))(print line)
         (setq ang-list (mapcar #'(lambda (x) (read-from-string x)) (reg-split " +" line)) )
         (when (= (length ang-list) 2)
             (push (elt ang-list 0) ankle-lang-list)
             (push (elt ang-list 1) ankle-rang-list))
         )));; end with open file
    (nreverse ankle-lang-list)(nreverse ankle-rang-list)

    (dotimes (i (length av-list))

      (setq av (elt av-list i))
      (setq av (concatenate float-vector (scale 1000 (subseq av 0 3)) (subseq av 3)))
      (send *m* :model :angle-vector av)

      (send *model-lfoot* :move-to (send *m* :model :rleg :end-coords :copy-worldcoords) :world)
      (send *model-lfoot* :rotate  (deg2rad (elt ankle-lang-list (* i rate))) :-y)
      (send *model-rfoot* :move-to (send *m* :model :lleg :end-coords :copy-worldcoords) :world)
      (send *model-rfoot* :rotate  (deg2rad (elt ankle-rang-list (* i rate))) :-y)
      ;; (warn "~a ~a~%"  (elt ankle-lang-list i) (elt ankle-rang-list i))

      (send *pickview* :draw-objects)
      
      (unix:usleep (* 1000 30))
      (x::window-main-one)

      );; end dotimes

    ))

;; (start)