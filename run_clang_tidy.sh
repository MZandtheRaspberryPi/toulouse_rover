for file in $(find ./build -name compile_commands.json) ; do
	run-clang-tidy -p $(dirname $file) ./src/toulouse_rover
done
