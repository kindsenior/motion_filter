;; bvhから人の動作のキーポーズを抽出する
#!/usr/bin/env roseus
#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(ros::roseus "hoge")

(load "~/prog/euslib/rbrain/eusbvh.l")
(load "package://roseus_tutorials/src/kinect-model-viewer.l")
(load "euslib/demo/k-kojima/bvh/perfume-bvh-robot-model.l")

(if (not (boundp '*gnuplot*))
    (setq *gnuplot* (gnuplot)))

(defun calc-vel (target)
  (let (pos)
    (setq pos (send (send *m* :model (elt target 0) :end-coords :copy-worldcoords) :worldpos)
          relpos (v- pos (elt target 1)))
    (push (norm relpos) (elt target 2))

    (setf (elt target 1) pos)))

(setq localmin-lists nil);; localmin-lists 極小リスト
(setq fm-offset 1);; bvhの読み込み開始フレーム

;; 手足腰の速度極小を求める
(defun find-localmin ()
  (let ()
    (setq localmin-lists nil);; 初期化

    ;; perfume モデル
    (setq bvhfile (concatenate string  "/home/" (unix::getenv "USER") "/prog/euslib/demo/k-kojima/bvh/bvhfiles/nocchi.bvh") )
    (setq *m* (load-mcd bvhfile :scale 10
                        :bvh-robot-model-class perfume-bvh-robot-model))
    ;; ;; kinect モデル
    ;; (setq *m* (load-mcd "bvhfiles/moon-walk.bvh" :scale 1 :bvh-robot-model-class kinect-bvh-robot-model))
    ;;*m* motion-capture-data
    ;; *m* . model 棒人間（bvhのロボットモデル）

    (objects (list  (send *m* :model) (make-cube 100 100 100)))

    ;; 速度極小を調べる対象のリスト (調査対象 直前の位置 速度リスト)
    (setq target-list (list (list :larm #f(0 0 0) nil)
                            (list :rarm #f(0 0 0) nil)
                            (list :lleg #f(0 0 0) nil)
                            (list :rleg #f(0 0 0) nil)));;腰位置の取得が必要

    ;; mcd読み込み
    (catch :import
      (dotimes (i (send *m* :frame-length))
        ;; (print i)
        (send *m* :frame (+ fm-offset i));;captureデータのフレーム番号をセット

        (dolist (val target-list) (calc-vel val));; targetの速度計算
        ;; (print target)

        (send (get *viewer* :pickviewer) :look-all)
        (if (select-stream (list *standard-input*) 1.0e-8) (throw :import nil))
        ;;catch内が nil を返して終了する

        (unix:usleep 10000)
        (x::window-main-one)))

    (dolist (val target-list) (setf (elt val 2) (reverse (elt val 2))) );; リストをリバース

    ;; フィルタ 今一、速度がガタガタ
    (dolist (val target-list)
      (let (tmp)
        (setq tmp (copy-tree (elt val 2)));; copy-tree リストをコピー eval 変数を評価
        (dotimes (i (1- (length tmp) ))
          (if (not (eq i 0))
              (setf (elt (elt val 2) i)
                    (/ (+ (elt tmp (1- i))
                          (elt tmp i)
                          (elt tmp (1+ i))) 3)))
          )))

    ;; 極小を求める
    (dolist (val target-list)
      (let ((tmp nil) (vel-list))
        (setq vel-list (elt val 2));; vel-list 速度リスト
        (dotimes (idx (length vel-list));; idx pivotのindex
          (let ((pre 0) (post 0) (range 1.0))
            (dotimes (i range);; pivotの前後rangeずつ平均をとる (返って極小が取れない)
              (if (and (> (- idx (1+ i)) 0) (< (+ idx (1+ i)) (length vel-list)))
                  (progn
                    (setq pre (+ pre (elt vel-list (- idx (1+ i)))))
                    (setq post (+ post (elt vel-list (+ idx (1+ i)))))
                    (if (and (< (elt vel-list idx) (/ pre range))
                             (< (elt vel-list idx) (/ post range)))
                        ;; (push (elt vel-list idx) tmp)
                        (push 1 tmp);; 1で極小
                      ;; (warn "found local min ~A ~A~%" idx (elt vel-list idx))
                      (push 0 tmp)));; 0で非極小
                (push 0 tmp)))))
        (push (reverse tmp) localmin-lists)
        ))
    (nreverse localmin-lists)
    ;; (print (elt localmin-lists 0))

    ;; gnuplot
    (send *gnuplot* :draw 
          (elt (elt target-list 0) 2)
          ;; (elt (elt target-list 1) 2)
          ;; (elt (elt target-list 2) 2)
          ;; (elt (elt target-list 3) 2)
          (elt localmin-lists 0)
          ;; (elt localmin-lists 1)
          ;; (elt localmin-lists 2)
          ;; (elt localmin-lists 3)
          :xrange (list 2 (length (elt (car target-list) 2)) )
          :yrange '(0 300)
          :title '("larm" "rarm" "lleg" "rleg")
          );; euslib/jsk/gnuplotlib.lを書き換えてmakeすると点なしてplotできる

    ;; (send *gnuplot* :draw
    ;;       (dolist (val target-list)
    ;;         (elt val 2)))
    ))

;; 速度極小からキーポーズを求める
(defun find-keypose ()
  (let ()
    ;; キーポーズ描画
    (setq a (make-cube 100 100 100))
    (setq b (make-cube 100 100 100))
    (send b :set-color :blue)
    (dotimes (idx (length (elt localmin-lists 0)) )
      (let ((nmin 0) (range 1) (nmin-thresh 2))
        ;; range localminを調べる幅 nmin-thresh キーポーズと判断する極小の数の閾値
        ;; 1:2 2:3

        (send *m* :frame (+ fm-offset idx));;captureデータのフレーム番号をセット

        ;; pivotの前後にいくつキーポーズ候補があるか確認
        (dolist (val localmin-lists)
          (catch :neighbor-search
            (dotimes (i range)
              (if (and (> (- idx (1+ i)) 0) (< (+ idx (1+ i)) (length val)))
                  (if (or (= (elt val (+ idx (1+ i))) 1) (= (elt val (- idx (1+ i))) 1) )
                      (progn
                        (setq nmin (1+ nmin))
                        (throw :neighbor-search nil)))))))

        (if (> nmin nmin-thresh)
            (progn 
              ;; (locate b (send *m* :model :head :worldpos))
              (objects (list  (send *m* :model) b))
              (unix:usleep 500000));; キーポーズと判断
          (objects (list  (send *m* :model) a)));; 非キーポーズと判断

        (send (get *viewer* :pickviewer) :look-all)
        
        (unix:usleep 10000)
        (x::window-main-one))
      )

    ))

;; (find-localmin)
;; (find-keypose)