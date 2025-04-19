#include "serializer.h"

std::vector<int8_t> Serializer::serialize(const BinarySerializationData &data) {
	const auto *dataPtr = reinterpret_cast<const int8_t *>(&data);
	return std::vector<int8_t>(
		reinterpret_cast<const int8_t*>(dataPtr),
		reinterpret_cast<const int8_t*>(dataPtr + BINARY_SERIALIZATION_DATA_SIZE)
	);
}

std::optional<BinarySerializationData> Serializer::deserialize(const std::vector<int8_t> &buffer) {
	if (buffer.size() != BINARY_SERIALIZATION_DATA_SIZE) {
		return std::nullopt;
	}

	BinarySerializationData data{};

	std::memcpy(&data, buffer.data(), BINARY_SERIALIZATION_DATA_SIZE);
	return data;
}

std::optional<BinarySerializationData> Serializer::deserialize(const int8_t *buffer, const size_t size) {
	if (size != BINARY_SERIALIZATION_DATA_SIZE) {
		return std::nullopt;
	}

	BinarySerializationData data{};
	std::memcpy(&data, buffer, BINARY_SERIALIZATION_DATA_SIZE);
	return data;
}