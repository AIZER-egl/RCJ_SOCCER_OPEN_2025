#ifndef SERIALIZER_H
#define SERIALIZER_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include <optional>
#include <cstring>

class Serializer {
public:
	Serializer() = delete;
	~Serializer() = delete;
	Serializer(const Serializer&) = delete;
	Serializer& operator=(const Serializer&) = delete;

	static std::vector<uint8_t> serialize(const BinarySerializationData& data);
	static std::optional<BinarySerializationData> deserialize(const std::vector<uint8_t>& buffer);
	static std::optional<BinarySerializationData> deserialize(const uint8_t* buffer, size_t size);
};

#endif //SERIALIZER_H
