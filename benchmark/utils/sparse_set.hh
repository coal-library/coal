#ifndef COAL_BENCH_UTILS_SPARSE_SET_HH
#define COAL_BENCH_UTILS_SPARSE_SET_HH

// includes
// std
#include <vector>
#include <cassert>

namespace coal {
namespace bench {
namespace utils {

template <typename _IndexType>
class SparseSet {
 public:
  using IndexType = _IndexType;

 public:
  SparseSet() = default;
  SparseSet(std::size_t max_val, std::size_t capacity)
      : m_dense(),
        m_sparse(max_val, DenseIndex(static_cast<IndexType>(capacity))) {
    m_dense.reserve(capacity);
  }

  void insert(std::size_t sparse_index) {
    assert(("SparseSet::insert fail: Out of range query",
            sparse_index < m_sparse.size()));

    SparseIndex si(static_cast<IndexType>(sparse_index));
    DenseIndex di(static_cast<IndexType>(size()));

    value(si, di);
    push_back(si);
  }

  bool contains(std::size_t sparse_index) const {
    assert(("SparseSet::contains fail: Out of range query",
            sparse_index < m_sparse.size()));
    // Check that the sparse value is pointing through a valid dense index
    // and that the dense value is equal to the sparse index
    SparseIndex si(static_cast<IndexType>(sparse_index));
    auto di = value(si);
    if (static_cast<std::size_t>(di.index) < size() &&
        value(di).index == si.index) {
      return true;
    }
    return false;
  }

  std::size_t size() const { return m_dense.size(); }

  void clear() { m_dense.clear(); }

 private:
  struct SparseIndex {
    SparseIndex() = default;
    explicit SparseIndex(IndexType i) : index(i) {}
    IndexType index;
  };
  struct DenseIndex {
    DenseIndex() = default;
    explicit DenseIndex(IndexType i) : index(i) {}
    IndexType index;
  };

  DenseIndex value(SparseIndex i) const { return m_sparse[i.index]; }
  SparseIndex value(DenseIndex i) const { return m_dense[i.index]; }

  void value(SparseIndex si, DenseIndex di) { m_sparse[si.index] = di; }
  void push_back(SparseIndex si) { m_dense.push_back(si); };

 private:
  std::vector<SparseIndex> m_dense;
  std::vector<DenseIndex> m_sparse;
};

}  // namespace utils
}  // namespace bench
}  // namespace coal

#endif  // ifndef COAL_BENCH_UTILS_SPARSE_SET_HH
