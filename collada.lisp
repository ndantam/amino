(in-package :robray)

(defvar *collada-dom*)

;;; Collada is right-handed

;;; TODO:
;;; - asset/unit
;;; - asset/up_axis



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; General Collada Parsing ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun collada-load (file)
  "Create the DOM for a Collada file."
  (cxml:parse-file file (cxml-dom:make-dom-builder)))

(defun collada-node-singleton (node tag-name)
  (let ((children (dom:get-elements-by-tag-name node tag-name)))
    (assert (= 1 (length children)))
    (elt children 0)))

(defun collada-node-text (node)
  (let ((children (dom:child-nodes node)))
    (assert (= 1 (length children)))
    (dom:data (elt children 0))))

(defun collada-text-split (node)
  (ppcre:split " " (collada-node-text node)))

(defun collada-node-integers (node)
  (map 'list #'parse-integer (collada-text-split node)))

(defun collada-node-floats (node)
  (map 'list #'parse-float (collada-text-split node)))


(defun collada-xml-id-map (dom)
  (let ((hash (make-hash-table :test #'equal)))
    (labels ((visit (node)
               (when (and (dom:element-p node)
                          (dom:has-attribute node "id"))
                 (setf (gethash (dom:get-attribute node "id") hash)
                       node))
               (map nil #'visit (dom:child-nodes node))))
      (visit dom))
    hash))

(defun collada-lookup (dom id &optional (id-map (collada-xml-id-map dom)))
  (assert (eql #\# (aref id 0)))
  (gethash (subseq id 1)
           id-map))

(defun collada-group-list (list stride)
  (loop for x = list then (nthcdr stride x)
     while x
     collect (subseq x 0 stride)))

(defun collada-parse-source (dom node)
  (declare (ignore dom))
  (assert (string= (dom:tag-name node) "source"))
  (let* ((array-node (collada-node-singleton node "float_array"))
         (array-values (collada-node-floats array-node))
         (technique-node (collada-node-singleton node "technique_common"))
         (accessor-node (collada-node-singleton technique-node "accessor"))
         (stride (if (dom:has-attribute accessor-node "stride")
                     (parse-integer (dom:get-attribute accessor-node "stride"))
                     1)))
    (assert (= 3 stride))
    (collada-group-list array-values stride)))

(defun collada-parse-vertices (dom node)
  (assert (string= (dom:tag-name node) "vertices"))
  (let ((input (collada-node-singleton node "input")))
    (assert (and (dom:has-attribute input "semantic")
                 (string= (dom:get-attribute input "semantic")
                          "POSITION")))
    (collada-attr-id-parse dom input "source")))

(defun collada-parse-float (dom node)
  (declare (ignore dom))
  (assert (string= (dom:tag-name node) "float"))
  (parse-float (collada-node-text node)))


(defun collada-parse-color (dom node)
  (declare (ignore dom))
  (assert (string= (dom:tag-name node) "color"))
  (collada-node-floats node))

(defun collada-parse-input (dom node)
  (assert (string= (dom:tag-name node) "input"))
  (collada-attr-id-parse dom node "source"))

(defstruct collada-effect
  emission
  ambient
  diffuse
  specular
  shininess
  index-of-refraction)

(defun collada-parse-effect (dom node)
  (assert (string= (dom:tag-name node) "effect"))
  (let* ((profile (collada-node-singleton node "profile_COMMON"))
         (technique (collada-node-singleton profile "technique"))
         (phong (collada-node-singleton technique "phong"))
         (result (make-collada-effect)))
    (labels ((color-value (node)
               (collada-parse-color dom (collada-node-singleton node "color")))
             (float-value (node)
               (collada-parse-float dom (collada-node-singleton node "float"))))
      (loop for child across (dom:child-nodes phong)
         when (dom:element-p child)
         do
           (string-case (dom:tag-name child)
              ("emission"
               (setf (collada-effect-emission result)
                     (color-value child)))
              ("ambient"
               (setf (collada-effect-ambient result)
                     (color-value child)))
              ("diffuse"
               (setf (collada-effect-diffuse result)
                     (color-value child)))
              ("specular"
               (setf (collada-effect-specular result)
                     (color-value child)))
              ("shininess"
               (setf (collada-effect-shininess result)
                     (float-value child)))
              ("index_of_refraction"
               (setf (collada-effect-index-of-refraction result)
                     (float-value child)))
              (otherwise
               (format *error-output* "Unknown effect tag: ~A" child)))))
    result))


(defun collada-parse-material (dom node)
  (assert (string= (dom:tag-name node) "material"))
  (let ((instance-effect (collada-node-singleton node "instance_effect")))
    (collada-attr-id-parse dom instance-effect "url")))


(defun collada-parse (dom node)
  (let ((tag (dom:tag-name node)))
    (string-case (dom:tag-name node)
      ("vertices"
       (collada-parse-vertices dom node))
      ("source"
       (collada-parse-source dom node))
      ("input"
       (collada-parse-input dom node))
      ("effect"
       (collada-parse-effect dom node))
      ("color"
       (collada-parse-color dom node))
      ("float"
       (collada-parse-float dom node))
      ("material"
       (collada-parse-material dom node))
      (otherwise
       (error "Unknown tag type: ~A" tag)))))

(defun collada-id-parse (dom id)
  (collada-parse dom (collada-lookup dom id)))

(defun collada-attr-id-parse (dom node attribute)
  (assert (dom:has-attribute node attribute))
  (collada-parse dom (collada-lookup dom (dom:get-attribute node attribute))))


;;;;;;;;;;;;;;;;;;;;;;;;;
;;; POVRAY CONVERSION ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;

(defun collada-povray-mangle (collada-identifier)
  (substitute #\_ #\- collada-identifier))

(defun collada-povray-geometry (dom geometry-node modifiers)
  (let* ((mesh  (collada-node-singleton geometry-node "mesh"))
         (polylist  (collada-node-singleton mesh "polylist"))
         (prim  (collada-node-integers (collada-node-singleton polylist "p")))
         (vcount  (collada-node-integers (collada-node-singleton polylist "vcount")))
         (inputs (dom:get-elements-by-tag-name polylist "input"))
         (vertices)
         (normals))
    ;; check vcount
    (dolist (c vcount)
      (assert (= 3 c) () "Cannot handle non-triangular polygons."))
    ;; Parse inputs
    (loop for input across inputs
       for semantic = (dom:get-attribute input "semantic")
       for value = (collada-parse dom input)
       do
         (cond ((string= semantic "VERTEX")
                (setq vertices value))
               ((string= semantic "NORMAL")
                (setq normals value))
               (t (error "Unknown input 'semantic' attribute: ~A" semantic))))
    ;; Parse Prim
    (let ((vertex-indices (collada-group-list (loop for rest = prim then (cddr rest)
                                                 while rest
                                                 collect (car rest))
                                              3))
          (normal-indices (collada-group-list (loop for rest = (cdr prim) then (cddr rest)
                                                 while rest
                                                 collect (car rest))
                                              3)))
      (assert (dom:has-attribute geometry-node "name"))
      (pov-declare (collada-povray-mangle (dom:get-attribute geometry-node "name"))
                   (pov-mesh2 :vertex-vectors (map 'list #'pov-float-vector vertices)
                              :normal-vectors (map 'list #'pov-float-vector normals)
                              :face-indices (map 'list #'pov-integer-vector vertex-indices)
                              :normal-indices (map 'list #'pov-integer-vector normal-indices)
                             :modifiers modifiers
                             )))))

               ;(map 'list (lambda (x) (apply #'pov-vector x)) normals)
    ;(print (list vertices normals))))


;; #S(COLLADA-EFFECT
;;    :EMISSION (0.0d0 0.0d0 0.0d0 1.0d0)
;;    :AMBIENT (0.0d0 0.0d0 0.0d0 1.0d0)
;;    :DIFFUSE (0.64d0 0.64d0 0.64d0 1.0d0)
;;    :SPECULAR (0.5d0 0.5d0 0.5d0 1.0d0)
;;    :SHININESS 50.0d0
;;    :INDEX-OF-REFRACTION 1.0d0)

(defun collada-povray-material (dom node)
  (let ((material (collada-parse dom node))
        (finishes))
    (labels ((avg-rgb (item)
               (pov-float (/ (+ (first item)
                                (second item)
                                (third item))
                             3d0)))
             (add-finish (name finish)
               (push (pov-item name finish)
                     finishes)))
      ;; TODO: emission, index-of-refraction
    (when (collada-effect-ambient material)
      (add-finish "ambient" (pov-rgb (collada-effect-ambient material))))
    (when (collada-effect-diffuse material)
      (add-finish "diffuse" (avg-rgb (collada-effect-diffuse material))))
    (when (collada-effect-shininess material)
      (add-finish "reflection" (pov-float (/ (collada-effect-shininess material)
                                             100d0))))
    (when (collada-effect-specular material)
      (add-finish "specular" (avg-rgb (collada-effect-specular material))))
    (pov-finish finishes))))

(defun collada-povray-visual-scene (dom node)
  (assert (string= "library_visual_scenes" (dom:tag-name node)))
  (loop
     for child-node across (dom:get-elements-by-tag-name node "node")
     for instance-geometry = (dom:get-elements-by-tag-name child-node "instance_geometry")
     unless (zerop (length instance-geometry))
     collect
       (progn
         (assert (= 1 (length instance-geometry)))
         (let ((instance-geometry (aref instance-geometry 0)))
           (assert (dom:has-attribute instance-geometry "url"))
           (let* ((geometry-node (collada-lookup dom (dom:get-attribute instance-geometry "url")))
                  (instance-material (dom:get-elements-by-tag-name instance-geometry "instance_material"))
                  (modifiers))
             ;; maybe set material
             (unless (zerop (length instance-material))
               (let ((instance-material (aref instance-material 0)))
                 (when (dom:has-attribute instance-material "target")
                   (push (collada-povray-material dom
                                                  (collada-lookup dom (dom:get-attribute instance-material
                                                                                         "target")))
                         modifiers))))
             (collada-povray-geometry dom geometry-node modifiers))))))

(defun collada-povray (&key
                         (dom *collada-dom*)
                         file)
  (let ((result (pov-sequence
                 (let* ((node (collada-node-singleton dom "library_visual_scenes")))
                   (collada-povray-visual-scene dom node)))))
    (if file
        (with-open-file (s file :direction :output :if-exists :supersede)
          (print result s)
          nil)
        result)))
