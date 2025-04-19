#include "serializer.h"

std::vector<uint8_t> Serializer::serialize(const BinarySerializationData &data) {
	const auto *dataPtr = reinterpret_cast<const uint8_t *>(&data);
	return std::vector<uint8_t>(dataPtr, dataPtr + BINARY_SERIALIZATION_DATA_SIZE);
}

std::optional<BinarySerializationData> Serializer::deserialize(const std::vector<uint8_t> &buffer) {
	if (buffer.size() != BINARY_SERIALIZATION_DATA_SIZE) {
		return std::nullopt;
	}

	BinarySerializationData data{};

	std::memcpy(&data, buffer.data(), BINARY_SERIALIZATION_DATA_SIZE);
	return data;
}

std::optional<BinarySerializationData> Serializer::deserialize(const uint8_t *buffer, const size_t size) {
	if (size != BINARY_SERIALIZATION_DATA_SIZE) {
		return std::nullopt;
	}

	BinarySerializationData data{};
	std::memcpy(&data, buffer, BINARY_SERIALIZATION_DATA_SIZE);
	return data;
}