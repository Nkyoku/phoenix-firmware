SRC_MAIN_DIR = src/main/scala
SRC_TEST_DIR = src/test/scala
OUT_DIR = generated

all : $(OUT_DIR)/QuadratureDecoder.v

test-all : 
	sbt "test:runMain QuadratureDecoder --target-dir $(OUT_DIR)"

clean :
	rmdir /s /q generated

$(OUT_DIR)/QuadratureDecoder.v : $(SRC_MAIN_DIR)/QuadratureDecoder.scala
	sbt "runMain QuadratureDecoder --target-dir $(OUT_DIR)"
