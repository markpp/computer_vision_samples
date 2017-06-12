# annotation_prep

Since the misc. class isn't annotated in the ORG_ANNO files, it needs to be extrapolated based on what is left. The program also generate annotations with a 1 pixel don't care border around each class.

The program reads the original 3 class annotations and the depth map in order to produce the complete 4 class annotation grayscale image.
