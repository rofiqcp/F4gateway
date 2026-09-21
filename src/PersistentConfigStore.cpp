#include "PersistentConfigStore.h"
#include "McuHal.h"

#include <cstring>

uint32_t PersistentConfigStore::crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (length-- != 0U) {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
      crc = (crc >> 1U) ^ (0xEDB88320UL & (0U - (crc & 1U)));
  }
  return crc ^ 0xFFFFFFFFUL;
}

bool PersistentConfigStore::erased(const Record &record) {
  const auto *p = reinterpret_cast<const uint8_t *>(&record);
  for (size_t i = 0U; i < sizeof(record); ++i)
    if (p[i] != 0xFFU) return false;
  return true;
}

bool PersistentConfigStore::valid(const Record &record) {
  return record.magic == kMagic && record.format == kFormat &&
         record.length > 0U && record.length <= kMaxValueBytes &&
         record.key != 0U && record.commit_word == kCommitWord &&
         record.crc32 == crc32(reinterpret_cast<const uint8_t *>(&record), 40U);
}

uint32_t PersistentConfigStore::totalSlots() const {
  return (kStorageLimit - kStorageBase) / sizeof(Record);
}

bool PersistentConfigStore::scan() {
  next_slot_ = 0U;
  next_sequence_ = 1U;
  invalid_records_ = 0U;
  const uint32_t slots = totalSlots();
  for (uint32_t i = 0U; i < slots; ++i) {
    Record record{};
    std::memcpy(&record,
                reinterpret_cast<const void *>(kStorageBase + i * sizeof(Record)),
                sizeof(record));
    if (erased(record)) {
      next_slot_ = i;
      storage_full_ = false;
      return true;
    }
    next_slot_ = i + 1U;
    if (!valid(record)) {
      ++invalid_records_;
      continue;
    }
    if (record.sequence >= next_sequence_)
      next_sequence_ = record.sequence + 1U;
  }
  storage_full_ = true;
  return true;
}

bool PersistentConfigStore::begin() {
  storage_ok_ = true;
  storage_full_ = false;
  writes_ = 0U;
  if (!scan()) {
    storage_ok_ = false;
    return false;
  }
  return true;
}

bool PersistentConfigStore::latest(uint16_t key, Record &record) const {
  bool found = false;
  uint32_t best_sequence = 0U;
  for (uint32_t i = 0U; i < next_slot_; ++i) {
    Record candidate{};
    std::memcpy(&candidate,
                reinterpret_cast<const void *>(kStorageBase + i * sizeof(Record)),
                sizeof(candidate));
    if (!valid(candidate) || candidate.key != key) continue;
    if (!found || candidate.sequence >= best_sequence) {
      record = candidate;
      best_sequence = candidate.sequence;
      found = true;
    }
  }
  return found;
}

bool PersistentConfigStore::read(uint16_t key, void *out, uint8_t length) const {
  if (key == 0U || out == nullptr || length == 0U || length > kMaxValueBytes)
    return false;
  Record record{};
  if (!latest(key, record) || record.length != length) return false;
  std::memcpy(out, record.data, length);
  return true;
}

bool PersistentConfigStore::readU32(uint16_t key, uint32_t &value) const {
  return read(key, &value, sizeof(value));
}

bool PersistentConfigStore::writeU32(uint16_t key, uint32_t value) {
  return write(key, &value, sizeof(value));
}

bool PersistentConfigStore::write(uint16_t key, const void *data, uint8_t length) {
  if (!storage_ok_ || storage_full_ || key == 0U || data == nullptr ||
      length == 0U || length > kMaxValueBytes)
    return false;

  Record current{};
  if (latest(key, current) && current.length == length &&
      std::memcmp(current.data, data, length) == 0)
    return true;

  if (next_slot_ >= totalSlots()) {
    storage_full_ = true;
    return false;
  }

  Record record{};
  std::memset(&record, 0xFF, sizeof(record));
  record.magic = kMagic;
  record.format = kFormat;
  record.length = length;
  record.key = key;
  record.reserved = 0xFFFFU;
  record.sequence = next_sequence_;
  std::memcpy(record.data, data, length);
  record.crc32 = crc32(reinterpret_cast<const uint8_t *>(&record), 40U);
  record.commit_word = kCommitWord;

  const uint32_t address = kStorageBase + next_slot_ * sizeof(Record);
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  bool ok = true;
#if defined(BOARD_F103C8)
  // STM32F1 medium-density flash programs 16-bit halfwords. Keep the commit
  // word last so a power loss can never turn a partial record into a valid one.
  for (uint32_t offset = 0U; offset < 44U && ok; offset += 2U) {
    uint16_t halfword = 0xFFFFU;
    std::memcpy(&halfword, reinterpret_cast<const uint8_t *>(&record) + offset,
                sizeof(halfword));
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + offset,
                           halfword) == HAL_OK;
  }
#else
  for (uint32_t offset = 0U; offset < 44U && ok; offset += 4U) {
    uint32_t word = 0xFFFFFFFFUL;
    std::memcpy(&word, reinterpret_cast<const uint8_t *>(&record) + offset,
                sizeof(word));
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + offset, word) == HAL_OK;
  }
#endif

  if (ok)
    ok = std::memcmp(reinterpret_cast<const void *>(address), &record, 44U) == 0;
  if (ok) {
#if defined(BOARD_F103C8)
    const uint16_t commitLo = static_cast<uint16_t>(record.commit_word & 0xFFFFU);
    const uint16_t commitHi = static_cast<uint16_t>(record.commit_word >> 16U);
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 44U, commitLo) == HAL_OK;
    if (ok)
      ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 46U, commitHi) == HAL_OK;
#else
    uint32_t commit = record.commit_word;
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + 44U, commit) == HAL_OK;
#endif
  }
  (void)HAL_FLASH_Lock();
  __DSB();

  ++next_slot_;
  if (next_slot_ >= totalSlots()) storage_full_ = true;
  if (!ok || std::memcmp(reinterpret_cast<const void *>(address), &record,
                         sizeof(record)) != 0) {
    ++invalid_records_;
    storage_ok_ = false;
    return false;
  }

  ++writes_;
  ++next_sequence_;
  if (next_sequence_ == 0U) next_sequence_ = 1U;
  return true;
}
