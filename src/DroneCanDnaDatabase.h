#pragma once
#ifdef NEO3PRO
#include <cstddef>
#include <cstdint>

class DroneCanDnaDatabase {
public:
  static constexpr uint8_t kMaxNodeId = 125U;
  static constexpr uint32_t kStorageBase = 0x0807E000UL; // compact 8 KiB DNA journal at end of persistent sector
  static constexpr uint32_t kStorageLimit = 0x08080000UL;

  bool begin(uint8_t own_node_id, const uint8_t *own_uid, uint8_t own_uid_len);
  bool isRegistered(uint8_t node_id) const;
  bool uidMatchesNode(uint8_t node_id, const uint8_t *uid, uint8_t uid_len) const;
  uint8_t findNodeId(const uint8_t *uid, uint8_t uid_len) const;
  bool handleNodeInfo(uint8_t source_node_id, const uint8_t unique_id[16]); // true => duplicate
  uint8_t handleAllocation(const uint8_t unique_id[16]);

  uint16_t registeredCount() const { return registered_count_; }
  uint32_t journalWrites() const { return journal_writes_; }
  uint32_t invalidJournalRecords() const { return invalid_journal_records_; }
  uint32_t usedJournalSlots() const { return next_slot_; }
  uint32_t totalJournalSlots() const;
  bool storageOk() const { return storage_ok_; }
  bool storageFull() const { return storage_full_; }

  struct NodeRecord {
    uint8_t uid_hash[6]{};
    uint8_t crc{0U};
  };

private:
  static constexpr uint16_t kJournalMagic = 0xAC01U;
  static constexpr uint8_t kJournalFormat = 1U;
  static constexpr uint8_t kFlagRegister = 1U;
  static constexpr uint32_t kCommitWord = 0xA5C39E71UL;
  static constexpr uint64_t kFnvOffset64 = 14695981039346656037ULL;
  static constexpr uint64_t kFnvPrime64 = 1099511628211ULL;

#pragma pack(push,1)
  struct JournalRecord {
    uint16_t magic;
    uint8_t format;
    uint8_t node_id;
    uint8_t uid_hash[6];
    uint8_t uid_crc;
    uint8_t flags;
    uint16_t sequence;
    uint8_t journal_crc;
    uint8_t reserved;
    uint32_t commit_word;
  };
#pragma pack(pop)
  static_assert(sizeof(JournalRecord) == 20U, "DNA journal record must remain 20 bytes");

  static uint8_t crc8(const uint8_t *data, uint8_t len);
  static NodeRecord computeUidHash(const uint8_t *uid, uint8_t len);
  static bool sameRecordHash(const NodeRecord &a, const NodeRecord &b);
  static bool recordValid(const NodeRecord &record);
  bool scanJournal();
  bool appendRegistration(uint8_t node_id, const NodeRecord &record);
  void applyRegistration(uint8_t node_id, const NodeRecord &record);
  bool registerUid(uint8_t node_id, const uint8_t *uid, uint8_t uid_len);

  NodeRecord records_[kMaxNodeId + 1U]{};
  bool registered_[kMaxNodeId + 1U]{};
  uint16_t registered_count_{0U};
  uint32_t next_slot_{0U};
  uint16_t next_sequence_{1U};
  uint32_t journal_writes_{0U};
  uint32_t invalid_journal_records_{0U};
  bool storage_ok_{true};
  bool storage_full_{false};
};
#endif
