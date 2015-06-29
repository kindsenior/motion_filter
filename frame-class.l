(defclass frame :slots (time idx
                              lfoot-state rfoot-state lhand-state rhand-state
                              waist-coord lfoot-coord rfoot-coord lhand-coord rhand-coord ltoe-coord rtoe-coord
                              waist-vel   lfoot-vel   rfoot-vel   lhand-vel   rhand-vel   ltoe-vel   rtoe-vel
                              waist-yawvel lfoot-yawvel rfoot-yawvel
                              zmp-pos angle-vector joint-pos-list
                              keypose-flg-list))

(defmethod frame
  (:init
   ()
   (setq lfoot-state -1 rfoot-state -1 lhand-state -1 rhand-state -1 keypose-flg-list nil)
   )
  (:set-idx (idx_) (setq idx idx_))
  (:idx (&optional idx_)
        (if idx_  (send self :set-idx idx_) (eval idx)) )

  (:set-time (time_) (setq time time_))
  (:time (&optional time_)
         (if time_ (send self :set-time time_) (eval time)))

  ;; 座標
  (:set-coord
   (limb coord_)
   (cond
    ((equal limb :waist) (setq waist-coord coord_))
    ((equal limb :lleg) (setq lfoot-coord coord_))
    ((equal limb :rleg) (setq rfoot-coord coord_))
    ((equal limb :larm) (setq lhand-coord coord_))
    ((equal limb :rarm) (setq rhand-coord coord_))
    ((equal limb :lleg-toe) (setq ltoe-coord coord_))
    ((equal limb :rleg-toe) (setq rtoe-coord coord_))
    ) )
  (:get-coord
   (limb)
   (cond
    ((equal limb :waist) waist-coord)
    ((equal limb :lleg) lfoot-coord)
    ((equal limb :rleg) rfoot-coord)
    ((equal limb :larm) lhand-coord)
    ((equal limb :rarm) rhand-coord)
    ((equal limb :lleg-toe) ltoe-coord)
    ((equal limb :rleg-toe) rtoe-coord)
    ) )
  (:coord (limb &optional coord_) (if coord_ (send self :set-coord limb coord_) (send self :get-coord limb)) )
  (:waist-coord
   (&optional coord_)
   (if coord_ (send self :set-coord :waist coord_) waist-coord))
  (:lfoot-coord
   (&optional coord_)
   (if coord_ (send self :set-coord :lleg coord_) lfoot-coord))
  (:rfoot-coord
   (&optional coord_)
   (if coord_ (send self :set-coord :rleg coord_) rfoot-coord))
  (:lhand-coord
   (&optional coord_)
   (if coord_ (send self :set-coord :larm coord_) lhand-coord))
  (:rhand-coord
   (&optional coord_)
   (if coord_ (send self :set-coord :rarm coord_) rhand-coord))

  ;; 位置
  (:set-pos
   (limb pos_)
   (cond
    ((equal limb :waist) (setq waist-coord (make-coords :pos pos_ :rot (send waist-coord :worldrot))) )
    ((equal limb :lleg)  (setq lfoot-coord (make-coords :pos pos_ :rot (send lfoot-coord :worldrot))) )
    ((equal limb :rleg)  (setq rfoot-coord (make-coords :pos pos_ :rot (send rfoot-coord :worldrot))) )
    ((equal limb :larm)  (setq lhand-coord (make-coords :pos pos_ :rot (send lhand-coord :worldrot))) )
    ((equal limb :rarm)  (setq rhand-coord (make-coords :pos pos_ :rot (send rhand-coord :worldrot))) )
    ((equal limb :lleg-toe)  (setq ltoe-coord (make-coords :pos pos_ :rot (send ltoe-coord :worldrot))) )
    ((equal limb :rleg-toe)  (setq rtoe-coord (make-coords :pos pos_ :rot (send rtoe-coord :worldrot))) )
    ) )
  (:get-pos
   (limb)
   (cond
    ((equal limb :waist) (send (send waist-coord :copy-worldcoords) :worldpos) )
    ((equal limb :lleg)  (send (send lfoot-coord :copy-worldcoords) :worldpos) )
    ((equal limb :rleg)  (send (send rfoot-coord :copy-worldcoords) :worldpos) )
    ((equal limb :larm)  (send (send lhand-coord :copy-worldcoords) :worldpos) )
    ((equal limb :rarm)  (send (send rhand-coord :copy-worldcoords) :worldpos) )
    ((equal limb :lleg-toe)  (send (send ltoe-coord :copy-worldcoords) :worldpos) )
    ((equal limb :rleg-toe)  (send (send rtoe-coord :copy-worldcoords) :worldpos) )
    ) )
  (:pos (limb &optional pos_) (if pos_ (send self :set-pos limb pos_) (send self :get-pos limb)))
  (:waist-pos (&optional pos_) (if pos_ (send self :set-pos :waist pos_) (send waist-coord :worldpos)))
  (:lfoot-pos (&optional pos_) (if pos_ (send self :set-pos :lleg pos_) (send lfoot-coord :worldpos)))
  (:rfoot-pos (&optional pos_) (if pos_ (send self :set-pos :rleg pos_) (send rfoot-coord :worldpos)))
  (:lhand-pos (&optional pos_) (if pos_ (send self :set-pos :larm pos_) (send lhand-coord :worldpos)))
  (:rhand-pos (&optional pos_) (if pos_ (send self :set-pos :rarm pos_) (send rhand-coord :worldpos)))

  ;; 高さ
  (:set-z
   (limb z_)
   (let ( (pos_ (send self :pos limb)) )
     (cond
      ( (equal limb :waist)
        (setq waist-coord (make-coords :pos (float-vector (elt pos_ 0) (elt pos_ 1) z_) :rot (send waist-coord :worldrot))) )
      ( (equal limb :lleg)
        (setq lfoot-coord (make-coords :pos (float-vector (elt pos_ 0) (elt pos_ 1) z_) :rot (send lfoot-coord :worldrot))) )
      ( (equal limb :rleg)
        (setq rfoot-coord (make-coords :pos (float-vector (elt pos_ 0) (elt pos_ 1) z_) :rot (send rfoot-coord :worldrot))) )
      )
     ))
  (:z (limb &optional z_) (if z_ (send self :set-z limb z_) (elt (send self :pos limb) 2)))

  ;; 姿勢
  (:set-rot
   (limb rot_)
   (cond
    ((equal limb :waist) (setq waist-coord (make-coords :pos (send waist-coord :worldpos) :rot rot_)) )
    ((equal limb :lleg)  (setq lfoot-coord (make-coords :pos (send lfoot-coord :worldpos) :rot rot_)) )
    ((equal limb :rleg)  (setq rfoot-coord (make-coords :pos (send rfoot-coord :worldpos) :rot rot_)) )
    ) )
  (:get-rot
   (limb)
   (cond
    ((equal limb :waist) (send (send waist-coord :copy-worldcoords) :worldrot) )
    ((equal limb :lleg)  (send (send lfoot-coord :copy-worldcoords) :worldrot) )
    ((equal limb :rleg)  (send (send rfoot-coord :copy-worldcoords) :worldrot) )
    ) )
  (:rot (limb &optional rot_) (if rot_ (send self :set-rot limb rot_) (send self :get-rot limb)))
  (:waist-rot
   (&optional rot_)
   (if rot_ (send self :set-rot :waist rot_) (send waist-coord :worldrot)))
  (:lfoot-rot
   (&optional rot_)
   (if rot_ (send self :set-rot :lleg rot_) (send lfoot-coord :worldrot)))
  (:rfoot-rot
   (&optional rot_)
   (if rot_ (send self :set-rot :rleg rot_) (send rfoot-coord :worldrot)))

  ;; pitch
  (:set-pitch
   (limb pitch_)
   (let ( (rpy_ (car (send (send self :coord limb) :rpy-angle))) )
     (cond
      ( (equal limb :waist)
        (setq waist-coord (make-coords :pos (send self :pos limb) :rpy (float-vector (elt rpy_ 0) pitch_ (elt rpy_ 2)))) )
      ( (equal limb :lleg)
        (setq lfoot-coord (make-coords :pos (send self :pos limb) :rpy (float-vector (elt rpy_ 0) pitch_ (elt rpy_ 2)))) )
      ( (equal limb :rleg)
        (setq rfoot-coord (make-coords :pos (send self :pos limb) :rpy (float-vector (elt rpy_ 0) pitch_ (elt rpy_ 2)))) )
      )
     ))
  (:pitch (limb &optional pitch_) (if pitch_ (send self :set-pitch limb pitch_) (elt (car (send (send self :coord limb) :rpy-angle)) 1)))
  ;; (:pitch (limb) (rad2deg (calc-pitch (send self :coord limb))))


  ;; 速度
  (:set-vel
   (limb vel_)
   (cond
    ((equal limb :waist) (setq waist-vel vel_) )
    ((equal limb :lleg)  (setq lfoot-vel vel_) )
    ((equal limb :rleg)  (setq rfoot-vel vel_) )
    ((equal limb :larm)  (setq lhand-vel vel_) )
    ((equal limb :rarm)  (setq rhand-vel vel_) )
    ((equal limb :lleg-toe)  (setq ltoe-vel vel_) )
    ((equal limb :rleg-toe)  (setq rtoe-vel vel_) )
    ) )
  (:get-vel
   (limb)
   (cond
    ((equal limb :waist) (eval waist-vel) )
    ((equal limb :lleg)  (eval lfoot-vel) )
    ((equal limb :rleg)  (eval rfoot-vel) )
    ((equal limb :larm)  (eval lhand-vel) )
    ((equal limb :rarm)  (eval rhand-vel) )
    ((equal limb :lleg-toe)  (eval ltoe-vel) )
    ((equal limb :rleg-toe)  (eval rtoe-vel) )
    ) )
  (:vel (limb &optional vel_) (if vel_ (send self :set-vel limb vel_) (send self :get-vel limb)))
  (:waist-vel
   (&optional vel_)
   (if vel_ (send self :set-vel :waist vel_) waist-vel ))
  (:lfoot-vel
   (&optional vel_)
   (if vel_ (send self :set-vel :lleg vel_) lfoot-vel ))
  (:rfoot-vel
   (&optional vel_)
   (if vel_ (send self :set-vel :rleg vel_) rfoot-vel ))
  (:lhand-vel
   (&optional vel_)
   (if vel_ (send self :set-vel :larm vel_) lhand-vel ))
  (:rhand-vel
   (&optional vel_)
   (if vel_ (send self :set-vel :rarm vel_) rhand-vel ))
  (:ltoe-vel
   (&optional vel_)
   (if vel_ (send self :set-vel :lleg-toe vel_) ltoe-vel ))
  (:rtoe-vel
   (&optional vel_)
   (if vel_ (send self :set-vel :rleg-toe vel_) rtoe-vel ))

  ;; 角速度
  (:set-yawvel
   (limb vel_)
   (cond
    ((equal limb :waist) (setq waist-yawvel vel_) )
    ((equal limb :lleg)  (setq lfoot-yawvel vel_) )
    ((equal limb :rleg)  (setq rfoot-yawvel vel_) )
    ) )
  (:get-yawvel
   (limb)
   (cond
    ((equal limb :waist) (eval waist-yawvel) )
    ((equal limb :lleg)  (eval lfoot-yawvel) )
    ((equal limb :rleg)  (eval rfoot-yawvel) )
    ) )
  (:yawvel (limb &optional vel_) (if vel_ (send self :set-yawvel limb vel_) (send self :get-yawvel limb)))
  (:waist-yawvel
   (&optional vel_)
   (if vel_ (send self :set-yawvel :waist vel_) waist-yawvel ))
  (:lfoot-yawvel
   (&optional vel_)
   (if vel_ (send self :set-yawvel :lleg vel_) lfoot-yawvel ))
  (:rfoot-yawvel
   (&optional vel_)
   (if vel_ (send self :set-yawvel :rleg vel_) rfoot-yawvel ))


  ;; 関節角
  (:set-angle-vector (angle-vector_) (setq angle-vector angle-vector_))
  (:angle-vector 
   (&optional angle-vector_)
   (if angle-vector_ (send self :set-angle-vector angle-vector_) (copy-seq angle-vector)) )

  ;; zmp
  (:set-zmp (zmp_) (setq zmp-pos zmp_))
  (:zmp
   (&optional zmp_)
   (if zmp_ (send self :set-zmp  zmp_) (copy-seq zmp-pos)) )

  ;; 接地状態
  (:set-state
   (limb state_)
   (cond
    ((equal limb :lleg)  (setq lfoot-state state_) )
    ((equal limb :rleg)  (setq rfoot-state state_) )
    ((equal limb :larm)  (setq lhand-state state_) )
    ((equal limb :rarm)  (setq rhand-state state_) )
    ) )
  (:get-state
   (limb)
   (cond
    ((equal limb :lleg) (eval lfoot-state) )
    ((equal limb :rleg) (eval rfoot-state) )
    ((equal limb :larm) (eval lhand-state) )
    ((equal limb :rarm) (eval rhand-state) )
    ) )
  (:state (limb &optional state_) (if state_ (send self :set-state limb state_) (send self :get-state limb)))

  ;; 関節位置リスト
  (:joint-pos-list (&optional joint-pos-list_) (if joint-pos-list_ (setq joint-pos-list joint-pos-list_) (copy-seq joint-pos-list)) )
  (:set-joint-pos-list-from-av
   (av)
   (setq joint-pos-list nil)
   ;; (push-back (subseq av 0 3) joint-pos-list);; 腰
   (dotimes (i (/ (length av) 3)) (push-back (coerce (subseq av (* i 3) (* (1+ i) 3)) float-vector) joint-pos-list))
   )

  ;; キーポーズフラグ
  (:push-keypose-flg
   (limb)
   (push limb keypose-flg-list))
  (:get-keypose-flg-list
   ()
   (copy-seq keypose-flg-list))

  )
