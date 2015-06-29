#USER=eus
BVH_DIR=/home/jsk/k-kojima/share/bvh
BVH_FILE=bvh_files
SFX=tgz

all:
	scp $(USER)@aries.jsk.t.u-tokyo.ac.jp:$(BVH_DIR)/$(BVH_FILE).$(SFX) ./
	make ext
	make clean_tar

ext:
	tar xzf $(BVH_FILE).$(SFX)

clean_tar:
	rm -f $(BVH_FILE).$(SFX)

clean: clean_tar
	rm -f *~

dist_clean: clean
	rm -rf $(BVH_FILE)
