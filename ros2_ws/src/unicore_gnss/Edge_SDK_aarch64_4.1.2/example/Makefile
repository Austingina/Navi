TARGET = demo

# SDK 头文件在 ../include，静态库在 ../library
HEADER_PATH = -I../include/

# libtpservice 只提供静态库，链接后 demo 不再依赖 ../library，可单独拷走。
# -lssl -lcrypto -lpthread 是 libtpservice.a 的未解析依赖，不可去掉。
LDFLAGS := -L../library/
LDFLAGS += -ltpservice -lm -lpthread -lssl -lcrypto

CFLAGS += -std=c99 -Wall -Wextra -Werror -O2 -g

all: $(TARGET)

$(TARGET): demo.o
	$(CC) $^ -g -o $@ $(LDFLAGS)

demo.o: demo.c
	$(CC) -c $< -o $@ $(HEADER_PATH) $(CFLAGS)

clean:
	rm -f *.o $(TARGET)

.PHONY: all clean
