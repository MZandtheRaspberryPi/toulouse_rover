for file in $(find ./build -name compile_commands.json) ; do
	run-clang-tidy -fix -header-filter="$CATKIN_WS/src/toulouse_rover/.*" -p $(dirname $file)
done
