#pragma once

#include <cstddef>
#include <cstdint>

class PersistentConfigStore {
public:
  static constexpr uint32_t kStorageBase = 0x08064000UL;
  static constexpr uint32_t kStorageLimit = 0x0806C000UL;
  static constexpr uint8_t kMaxValueBytes = 28U;

  bool begin();
  bool read(uint16_t key, void *out, uint8_t length) const;
  bool write(uint16_t key, const void *data, uint8_t length);
  bool readU32(uint16_t key, uint32_t &value) const;
  bool writeU32(uint16_t key, uint32_t value);

  bool storageOk() const { return storage_ok_; }
  bool storageFull() const { return storage_full_; }
  uint32_t usedSlots() const { return next_slot_; }
  uint32_t totalSlots() const;
  uint32_t invalidRecords() const { return invalid_records_; }
  uint32_t writes() const { return writes_; }

private:
  static constexpr uint16_t kMagic = 0xEC01U;
  static constexpr uint8_t kFormat = 1U;
  static constexpr uint32_t kCommitWord = 0xC04D17EDUL;

#pragma pack(push, 1)
  struct Record {
    uint16_t magic;
    uint8_t format;
    uint8_t length;
    uint16_t key;
    uint16_t reserved;
    uint32_t sequence;
    uint8_t data[kMaxValueBytes];
    uint32_t crc32;
    uint32_t commit_word;
  };
#pragma pack(pop)
  static_assert(sizeof(Record) == 48U, "persistent config record must remain 48 bytes");

  static uint32_t crc32(const uint8_t *data, size_t length);
  static bool erased(const Record &record);
  static bool valid(const Record &record);
  bool scan();
  bool latest(uint16_t key, Record &record) const;

  uint32_t next_slot_{0U};
  uint32_t next_sequence_{1U};
  uint32_t invalid_records_{0U};
  uint32_t writes_{0U};
  bool storage_ok_{true};
  bool storage_full_{false};
};
