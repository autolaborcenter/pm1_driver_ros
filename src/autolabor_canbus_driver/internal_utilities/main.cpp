//
// Created by ydrml on 2019/2/28.
//

#include <iostream>
#include <assert.h>
#include "big_endian_transform.h"

int main() {
	std::vector<uint8_t> code{0, 0, 0, 99};
	
	auto decoded = autolabor::build<int>(code.data());
	auto encoded = autolabor::pack(decoded);
	
	assert(decoded == 99);
	
	for (int i = 0; i < 4; ++i)
		assert(encoded[i] == code[i]);
	
	return 0;
}
