CURRDIR=$(pwd)
for e in */ ; do
	echo "building $e"
	cd "$e"
	cd gcc
	make clean
	export TARGET_BOARD=BOARD_PCA10031
	export USE_BUTTONS=\"no\"
	export USE_RBC_MESH_SERIAL=\"no\"
	make
	
	make cleanobj
	export TARGET_BOARD=BOARD_PCA10028
	make
	
	make cleanobj
	export USE_BUTTONS=\"yes\"
	make

	make cleanobj
	export USE_BUTTONS=\"no\"
	export USE_RBC_MESH_SERIAL=\"yes\"
	make
	cd $CURRDIR
done

