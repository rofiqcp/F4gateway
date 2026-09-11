#include "DroneCanDnaDatabase.h"
#ifdef NEO3PRO
#include "stm32f4xx_hal.h"
#include <cstring>

namespace {
bool isErased(const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; ++i) if (p[i] != 0xFFU) return false;
  return true;
}
}

uint8_t DroneCanDnaDatabase::crc8(const uint8_t *data, uint8_t len) {
  // Same CRC-8/0x07 used by ArduPilot crc_crc8().
  uint8_t crc = 0U;
  while (len--) {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; ++bit)
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ 0x07U)
                          : static_cast<uint8_t>(crc << 1U);
  }
  return crc;
}

DroneCanDnaDatabase::NodeRecord DroneCanDnaDatabase::computeUidHash(const uint8_t *uid, uint8_t len) {
  NodeRecord record{};
  if (uid == nullptr || len == 0U) return record;
  uint64_t hash = kFnvOffset64;
  for (uint8_t i = 0U; i < len; ++i) { hash ^= static_cast<uint64_t>(uid[i]); hash *= kFnvPrime64; }
  // Intentionally match AP_DroneCAN_DNA_Server, including its historical 56-bit xor fold.
  hash = (hash >> 56U) ^ (hash & ((1ULL << 56U) - 1ULL));
  for (uint8_t i = 0U; i < 6U; ++i) record.uid_hash[i] = static_cast<uint8_t>(hash >> (8U * i));
  record.crc = crc8(record.uid_hash, sizeof(record.uid_hash));
  return record;
}

bool DroneCanDnaDatabase::sameRecordHash(const NodeRecord &a, const NodeRecord &b) {
  return std::memcmp(a.uid_hash, b.uid_hash, sizeof(a.uid_hash)) == 0;
}

bool DroneCanDnaDatabase::recordValid(const NodeRecord &record) {
  uint8_t zero[6]{};
  return std::memcmp(record.uid_hash, zero, sizeof(zero)) != 0 &&
         crc8(record.uid_hash, sizeof(record.uid_hash)) == record.crc;
}

uint32_t DroneCanDnaDatabase::totalJournalSlots() const {
  return (kStorageLimit - kStorageBase) / sizeof(JournalRecord);
}

void DroneCanDnaDatabase::applyRegistration(uint8_t node_id, const NodeRecord &record) {
  if (node_id == 0U || node_id > kMaxNodeId || !recordValid(record)) return;
  // ArduPilot register_uid() removes any old node-ID registration for this UID.
  for (uint8_t id = 1U; id <= kMaxNodeId; ++id) {
    if (id != node_id && registered_[id] && sameRecordHash(records_[id], record)) {
      registered_[id] = false;
      records_[id] = NodeRecord{};
      if (registered_count_ > 0U) --registered_count_;
    }
  }
  if (!registered_[node_id]) ++registered_count_;
  records_[node_id] = record;
  registered_[node_id] = true;
}

bool DroneCanDnaDatabase::scanJournal() {
  std::memset(records_, 0, sizeof(records_));
  std::memset(registered_, 0, sizeof(registered_));
  registered_count_ = 0U; next_slot_ = 0U; next_sequence_ = 1U;
  const uint32_t slots = totalJournalSlots();
  for (uint32_t i = 0U; i < slots; ++i) {
    const auto *raw = reinterpret_cast<const JournalRecord *>(kStorageBase + i * sizeof(JournalRecord));
    if (isErased(reinterpret_cast<const uint8_t *>(raw), sizeof(JournalRecord))) { next_slot_ = i; return true; }
    next_slot_ = i + 1U; // never reuse a partially programmed/torn slot
    JournalRecord jr{}; std::memcpy(&jr, raw, sizeof(jr));
    if (jr.magic != kJournalMagic || jr.format != kJournalFormat || jr.commit_word != kCommitWord ||
        jr.node_id == 0U || jr.node_id > kMaxNodeId || jr.flags != kFlagRegister ||
        crc8(reinterpret_cast<const uint8_t *>(&jr), 14U) != jr.journal_crc) {
      ++invalid_journal_records_; continue;
    }
    NodeRecord nr{}; std::memcpy(nr.uid_hash, jr.uid_hash, sizeof(nr.uid_hash)); nr.crc = jr.uid_crc;
    if (!recordValid(nr)) { ++invalid_journal_records_; continue; }
    applyRegistration(jr.node_id, nr);
    next_sequence_ = static_cast<uint16_t>(jr.sequence + 1U);
    if (next_sequence_ == 0U) next_sequence_ = 1U;
  }
  storage_full_ = true;
  return true;
}

bool DroneCanDnaDatabase::appendRegistration(uint8_t node_id, const NodeRecord &record) {
  if (node_id == 0U || node_id > kMaxNodeId || !recordValid(record) || next_slot_ >= totalJournalSlots()) {
    storage_full_ = next_slot_ >= totalJournalSlots(); return false;
  }
  JournalRecord jr{};
  jr.magic = kJournalMagic; jr.format = kJournalFormat; jr.node_id = node_id;
  std::memcpy(jr.uid_hash, record.uid_hash, sizeof(jr.uid_hash)); jr.uid_crc = record.crc;
  jr.flags = kFlagRegister; jr.sequence = next_sequence_; jr.reserved = 0xFFU;
  jr.journal_crc = crc8(reinterpret_cast<const uint8_t *>(&jr), 14U);
  jr.commit_word = kCommitWord;
  const uint32_t address = kStorageBase + next_slot_ * sizeof(JournalRecord);
  if (HAL_FLASH_Unlock() != HAL_OK) { storage_ok_ = false; return false; }
  bool ok = true;
  const uint32_t *words = reinterpret_cast<const uint32_t *>(&jr);
  // True two-phase commit: first program and verify all payload/CRC words, then
  // program the independent 32-bit commit marker as the final flash operation.
  for (uint8_t i = 0U; i < 4U && ok; ++i)
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + 4U*i, words[i]) == HAL_OK;
  if (ok) ok = std::memcmp(reinterpret_cast<const void *>(address), &jr, 16U) == 0;
  if (ok) ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + 16U, words[4]) == HAL_OK;
  (void)HAL_FLASH_Lock(); __DSB();
  if (ok) ok = std::memcmp(reinterpret_cast<const void *>(address), &jr, sizeof(jr)) == 0;
  if (!ok) { storage_ok_ = false; ++invalid_journal_records_; ++next_slot_; return false; }
  ++journal_writes_; ++next_slot_; ++next_sequence_; if (next_sequence_ == 0U) next_sequence_ = 1U;
  if (next_slot_ >= totalJournalSlots()) storage_full_ = true;
  applyRegistration(node_id, record);
  return true;
}

bool DroneCanDnaDatabase::registerUid(uint8_t node_id, const uint8_t *uid, uint8_t uid_len) {
  if (node_id == 0U || node_id > kMaxNodeId || uid == nullptr || uid_len == 0U) return false;
  const NodeRecord rec = computeUidHash(uid, uid_len);
  if (!recordValid(rec)) return false;
  if (registered_[node_id] && sameRecordHash(records_[node_id], rec)) return true;
  return appendRegistration(node_id, rec);
}

bool DroneCanDnaDatabase::begin(uint8_t own_node_id, const uint8_t *own_uid, uint8_t own_uid_len) {
  storage_ok_ = true; storage_full_ = false; invalid_journal_records_ = 0U; journal_writes_ = 0U;
  if (!scanJournal()) { storage_ok_ = false; return false; }
  // Same semantics as ArduPilot Database::init_server(): reserve server node ID persistently.
  if (findNodeId(own_uid, own_uid_len) != own_node_id && !registerUid(own_node_id, own_uid, own_uid_len)) return false;
  return true;
}

bool DroneCanDnaDatabase::isRegistered(uint8_t node_id) const {
  return node_id > 0U && node_id <= kMaxNodeId && registered_[node_id];
}

uint8_t DroneCanDnaDatabase::findNodeId(const uint8_t *uid, uint8_t uid_len) const {
  if (uid == nullptr || uid_len == 0U) return 0U;
  const NodeRecord rec = computeUidHash(uid, uid_len);
  // ArduPilot searches from MAX_NODE_ID downward.
  for (int id = kMaxNodeId; id > 0; --id)
    if (registered_[id] && sameRecordHash(records_[id], rec)) return static_cast<uint8_t>(id);
  return 0U;
}

bool DroneCanDnaDatabase::uidMatchesNode(uint8_t node_id, const uint8_t *uid, uint8_t uid_len) const {
  return isRegistered(node_id) && findNodeId(uid, uid_len) == node_id;
}

bool DroneCanDnaDatabase::handleNodeInfo(uint8_t source_node_id, const uint8_t unique_id[16]) {
  if (source_node_id == 0U || source_node_id > kMaxNodeId || unique_id == nullptr) return true;
  if (isRegistered(source_node_id)) return findNodeId(unique_id, 16U) != source_node_id;
  return !registerUid(source_node_id, unique_id, 16U); // storage failure is fail-closed
}

uint8_t DroneCanDnaDatabase::handleAllocation(const uint8_t unique_id[16]) {
  if (unique_id == nullptr || !storage_ok_) return 0U;
  uint8_t node_id = findNodeId(unique_id, 16U);
  if (node_id != 0U) return node_id;
  // UAVCAN v0 centralized allocator / ArduPilot: allocate highest free node ID first.
  for (int id = kMaxNodeId; id > 0; --id) {
    if (!registered_[id]) {
      node_id = static_cast<uint8_t>(id);
      return registerUid(node_id, unique_id, 16U) ? node_id : 0U;
    }
  }
  storage_full_ = true;
  return 0U;
}
#endif
