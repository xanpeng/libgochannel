run: async_channel sync_channel
	./sync_channel
	echo "\n"
	./async_channel

async_channel:
	g++ async_channel.cc -o async_channel -pthread --std=c++11

sync_channel:
	g++ sync_channel.cc -o sync_channel -pthread --std=c++11

clean:
	rm async_channel
	rm sync_channel
