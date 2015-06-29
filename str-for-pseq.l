;; bvh2pseqに必要なメソッド

;; １次元リストをpseq内の文字列に変換
(defun list2str (list name &optional (flg nil))
  (let (str)
    (setq str (concatenate string name ": [ "))
    (let ((i 0))
      (dolist (val list)
        (when (string= name "translation")
          (setq val (/ val 1000));; translationはmmからmに変更
          ;; (if (and flg (= i 2))  (setq val 0.0890143192));;接地足の高さを無理やり固定
          ;; (if (and flg (= i 2)) (setq val 0.095496));; 不要??
          (if (and flg (= i 2)) (setq val 0.104505021))
          )
        (when (and (string= name "rotation") flg);; 姿勢を無理やり水平に固定
          (case i ((2 5 6 7) (setq val 0))
                ((8) (setq val 1))))
        (setq str (concatenate string str
                               (string  val)
                               ", "))
        (incf i)))
    (setq str (string-right-trim ", " str))
    (setq str (concatenate string str " ]"))
    str
    ))

;;iklinks文字列のユニット作成
(defun unite-iklinks (trans-str rot-str name idx touch-flg)
  (let (str isbaselink istouching)
    (if (string= name "WAIST")
        (setq isbaselink "          isBaseLink: true~%"))
    (if touch-flg
        (setq istouching (concatenate string "          isTouching: true~%"
                                      "          partingDirection: [ 0, 0, 1 ]~%")))
    (setq str (concatenate string "        -~%"
                           "          name: " name "~%"
                           "          index: " (string idx) "~%"
                           isbaselink
                           "          " trans-str "~%"
                           "          " rot-str "~%"
                           istouching))
    str))

;;iklinks文字列の追加
(defun add-iklinks (iklinks name idx part coord &key (touch-flg nil) (offset-vec nil) (waist-z-offset 0) )
  (let (pos-list rot-list pos-str rot-str)
    ;; pos-list処理
    (cond ((not offset-vec)
           (setq pos-list (coerce (send *robot* part coord :worldpos) cons)))
          (t
           (if (not part)
               (progn
                 (setq offset-vec (send *robot* :rotate-vector offset-vec))
                 (setq pos-list (coerce (v+ (send *robot* :worldpos) offset-vec) cons)));; ロボットの座標系が腰座標系に一致するもの
             (if (not coord)
                 (progn 
                   (setq offset-vec (send *robot* part :rotate-vector offset-vec))
                   (setq pos-list (coerce (v+ (send *robot* part :worldpos) offset-vec) cons)))
               (progn (setq offset-vec (send *robot* part coord :rotate-vector offset-vec))
                      (setq pos-list (coerce (v+ (send *robot* part coord :worldpos) offset-vec) cons))))))
          )

    (setf (elt pos-list 2) (+ (elt pos-list 2) waist-z-offset));; ZMP補償のための腰高さのオフセット

    ;; rot-list処理
    (if (equal name "WAIST")
        (setq rot-list (coerce (subseq (send *robot* :worldrot) 0) cons))
      (setq rot-list (coerce (subseq (send *robot* part coord :worldrot) 0) cons)))

    (setq trans-str (list2str pos-list "translation" touch-flg))
    (setq rot-str (list2str rot-list "rotation" touch-flg))
    (setq iklinks (concatenate string iklinks (unite-iklinks  trans-str  rot-str name idx touch-flg)))
    iklinks
    ))
