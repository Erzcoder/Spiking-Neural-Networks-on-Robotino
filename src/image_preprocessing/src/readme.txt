image_prep_popul_conv.cpp
	Contains preprocessing + population coding using the 3 kernels (line representation) convolutions.

image_preprocessing_node.cpp
	Contains the original preprocessing behaviour with no spike encoding included.

image_preprocessing_node_v1.cpp
	Testing version, can be deleted.

img_preproc_pop.cpp
	Contains preprocessing + population based on weighted column wise summation.

sim_img_preproc_pop.cpp
	Simple image preprocessing+population coding, not really usefull, but legacy.

simple_image_preprocessing_node.cpp
	Simple image preprocessing, not really usefull, but legacy.

Remarqs:

-When using webcam different then the laptop's one, go to cv_camera/capture.cpp 
and update open(0) to open(1)
-Always adapt ros topics when switching from bags testing to real testing