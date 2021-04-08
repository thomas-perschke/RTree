#ifndef RTREE_H
#define RTREE_H

#include <algorithm>
#include <functional>
#include <stack>
#include <forward_list>
#include <unordered_map>
#include <vector>
#include <array>
#include <tuple>
#include <cmath>
#include <cassert>

#if __has_cpp_attribute(__cpp_lib_math_constants)
#include <numeric>
namespace
{
constexpr double Pi = std::numeric::pi;
}
#else
namespace
{
constexpr double Pi = acos(-1.);
}
#endif

#define ASSERT assert // RTree uses ASSERT( condition )

//very lazy workaround for windows issue when including windows.h
#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

//
// RTree.h
//

#define RTREE_TEMPLATE                                                        \
   template <class DATATYPE, class ELEMTYPE, int NUMDIMS, class ELEMTYPEREAL, \
             int TMAXNODES, int TMINNODES>
#define RTREE_QUAL \
   RTree<DATATYPE, ELEMTYPE, NUMDIMS, ELEMTYPEREAL, TMAXNODES, TMINNODES>

#define RTREE_USE_SPHERICAL_VOLUME // Better split classification, may be slower on some systems

/// \class RTree
/// Implementation of RTree, a multidimensional bounding rectangle tree.
/// Example usage: For a 3-dimensional tree use RTree<Object*, float, 3> myTree;
///
/// This modified, templated C++ version by Greg Douglas at Auran (http://www.auran.com)
///
/// Made C++17 compatible by Thomas Perschke
///
/// DATATYPE Referenced data, should be int, void*, obj* etc. no larger than sizeof<void*> and simple type
/// ELEMTYPE Type of element such as int or float
/// NUMDIMS Number of dimensions such as 2 or 3
/// ELEMTYPEREAL Type of element that allows fractional and large values such as float or double, for use in volume calcs
///
/// NOTES: Inserting and removing data requires the knowledge of its constant Minimal Bounding Rectangle.
template <class DATATYPE, class ELEMTYPE, int NUMDIMS,
          class ELEMTYPEREAL = ELEMTYPE, int TMAXNODES = 8,
          int TMINNODES = TMAXNODES / 2>
class RTree
{
   static_assert(std::is_floating_point<ELEMTYPEREAL>::value,
                 "'ELEMTYPEREAL' accepts floating-point types only");

 private:
   struct Node; // Fwd decl.  Used by other internal structs and iterator

 public:
   RTree();
   virtual ~RTree() {}

   /// Insert entry
   /// \param a_min Min of bounding rect
   /// \param a_max Max of bounding rect
   /// \param a_dataId  Id of data
   void Insert(const std::array<ELEMTYPE, NUMDIMS> &a_min,
               const std::array<ELEMTYPE, NUMDIMS> &a_max,
               const DATATYPE &                     a_dataId);

   /// Remove entry
   /// \param a_min Min of bounding rect
   /// \param a_max Max of bounding rect
   /// \param Id of data
   void Remove(const std::array<ELEMTYPE, NUMDIMS> &a_min,
               const std::array<ELEMTYPE, NUMDIMS> &a_max,
               const DATATYPE &                     a_dataId);

   /// Find all within search rectangle
   /// \param a_min Min of search bounding rect
   /// \param a_max Max of search bounding rect
   /// \param a_resultCallback Callback function to return result.  Callback should return 'true' to continue searching
   /// \return Returns the number of entries found

   int Search(const std::array<ELEMTYPE, NUMDIMS> & a_min,
              const std::array<ELEMTYPE, NUMDIMS> & a_max,
              std::function<bool(const DATATYPE &)> callback) const;

   using result_iterator = typename std::vector<DATATYPE>::iterator;

   /// Find all within search rectangle. NOT thread-safe !
   /// \param a_min Min of search bounding rect
   /// \param a_max Max of search bounding rect
   /// \param a_resultCallback Callback function to return result.  Callback should return 'true' to continue searching
   /// \return Returns pair of iterators [begin,end)
   std::tuple<result_iterator, result_iterator>
   Search(const std::array<ELEMTYPE, NUMDIMS> &a_min,
          const std::array<ELEMTYPE, NUMDIMS> &a_max, int max_count = 0) const;

   /// Remove all entries from tree
   void RemoveAll();

   /// Count the data elements in this container.
   size_t Count() const;

   /// Iterator is not remove safe.
   struct Tree_Iterator
   {

      using iterator_category = std::forward_iterator_tag;
      using difference_type   = std::ptrdiff_t;
      using value_type        = DATATYPE;
      using reference         = DATATYPE &;

      struct StackElement
      {
         int node_id;
         int m_branchIndex;
      };
      friend bool operator==(const StackElement &a, const StackElement &b)
      {
         return (a.node_id == b.node_id) &&
                (a.m_branchIndex == b.m_branchIndex);
      }

      Tree_Iterator(RTree *tree) : tree(tree), node_iterator(tree->Nodes.end())
      {
      }

      value_type operator*() const
      {
         auto curTos   = m_stack.top();
         node_iterator = tree->Nodes.find(curTos.node_id);
         return node_iterator->second.m_branch[curTos.m_branchIndex].m_data;
      }
      reference operator*()
      {
         auto curTos   = m_stack.top();
         node_iterator = tree->Nodes.find(curTos.node_id);
         return node_iterator->second.m_branch[curTos.m_branchIndex].m_data;
      }
      Tree_Iterator &operator++()
      {
         FindNextData();
         return *this;
      }

      Tree_Iterator operator++(int)
      {
         Tree_Iterator tmp = *this;
         ++(*this);
         return tmp;
      }

      friend bool operator==(const Tree_Iterator &a, const Tree_Iterator &b)
      {
         return (a.tree == b.tree) && (a.m_stack == b.m_stack);
      };
      friend bool operator!=(const Tree_Iterator &a, const Tree_Iterator &b)
      {
         return (a.tree != b.tree) || (a.m_stack != b.m_stack);
      };

      /// Get the bounds for this node
      void GetBounds(std::array<ELEMTYPE, NUMDIMS> &a_min,
                     std::array<ELEMTYPE, NUMDIMS> &a_max)
      {
         auto          curTos    = m_stack.top();
         const auto &  m_node    = tree->Nodes.at(curTos.node_id);
         const Branch &curBranch = m_node.m_branch[curTos.m_branchIndex];

         for (int index = 0; index < NUMDIMS; ++index)
         {
            a_min[index] = curBranch.m_rect.m_min[index];
            a_max[index] = curBranch.m_rect.m_max[index];
         }
      }

      RTree *                                          tree;
      typename std::unordered_map<int, Node>::iterator node_iterator;

      bool IsNull() const { return m_stack.empty(); }
      /// Find the next data element in the tree (For internal use only)
      bool FindNextData()
      {
         for (;;)
         {
            if (IsNull())
            {
               return false;
            }
            StackElement curTos =
                Pop(); // Copy stack top cause it may change as we use it
            const auto &m_node = tree->Nodes.at(curTos.node_id);

            if (m_node.IsLeaf())
            {
               // Keep walking through data while we can
               if (curTos.m_branchIndex + 1 < m_node.m_count)
               {
                  // There is more data, just point to the next one
                  Push(m_node.ID, curTos.m_branchIndex + 1);
                  return true;
               }
               // No more data, so it will fall back to previous level
            }
            else
            {
               if (curTos.m_branchIndex + 1 < m_node.m_count)
               {
                  // Push sibling on for future tree walk
                  // This is the 'fall back' node when we finish with the current level
                  Push(m_node.ID, curTos.m_branchIndex + 1);
               }
               // Since cur node is not a leaf, push first of next level to get deeper into the tree
               // Node* nextLevelnode = curTos.m_node->m_branch[curTos.m_branchIndex].m_child;
               const auto &nextLevelnode = tree->Nodes.at(
                   m_node.m_branch[curTos.m_branchIndex].child_ID);
               Push(nextLevelnode.ID, 0);

               // If we pushed on a new leaf, exit as the data is ready at TOS
               if (nextLevelnode.IsLeaf())
               {
                  return true;
               }
            }
         }
      }

      /// Push node and branch onto iteration stack (For internal use only)
      void Push(int node_id, int a_branchIndex)
      {
         m_stack.emplace(StackElement{node_id, a_branchIndex});
      }

      /// Pop element off iteration stack (For internal use only)
      StackElement Pop()
      {
         ASSERT(!m_stack.empty());
         StackElement result = m_stack.top();
         m_stack.pop();
         return result;
      }
      std::stack<StackElement> m_stack;

      friend class
          RTree; // Allow hiding of non-public functions while allowing manipulation by logical owner
   };

   Tree_Iterator begin()
   {
      Tree_Iterator it(this);
      GetFirst(it);
      return it;
   }

   Tree_Iterator end()
   {
      Tree_Iterator it(this);
      return it;
   }

 private:
   static constexpr int MAXNODES = TMAXNODES;
   static constexpr int MINNODES = TMINNODES;

   void GetFirst(Tree_Iterator &a_it)
   {
      int id = root_ID;

      while (true)
      {
         const auto &first = Nodes.at(id);
         if (first.IsInternalNode() && first.m_count > 1)
         {
            a_it.Push(first.ID, 1); // Descend sibling branch later
         }
         else if (first.IsLeaf())
         {
            if (first.m_count)
            {
               a_it.Push(first.ID, 0);
            }
            break;
         }
         id = first.m_branch[0].child_ID;
      }
   }

   /// Minimal bounding rectangle (n-dimensional)
   struct Rect
   {
      std::array<ELEMTYPE, NUMDIMS> m_min; ///< Min dimensions of bounding box
      std::array<ELEMTYPE, NUMDIMS> m_max; ///< Max dimensions of bounding box
      Rect() = default;
      Rect(const std::array<ELEMTYPE, NUMDIMS> &a_min,
           const std::array<ELEMTYPE, NUMDIMS> &a_max)
          : m_min(a_min), m_max(a_max)
      {
      }
      friend bool operator==(const Rect &a, const Rect &b)
      {
         return (a.m_min == b.m_min) && (a.m_max == b.m_max);
      }
      friend bool operator!=(const Rect &a, const Rect &b) { return !(a == b); }
   };

   /// May be data or may be another subtree
   /// The parents level determines this.
   /// If the parents level is 0, then this is data
   struct Branch
   {
      Rect     m_rect;   ///< Bounds
      int      child_ID; ///< Child Id
      DATATYPE m_data;   ///< Data Id

      friend bool operator==(const Branch &a, const Branch &b)
      {
         return (a.m_rect == b.m_rect) && (a.child_ID == b.child_ID) &&
                (a.m_data == b.m_data);
      }
      friend bool operator!=(const Branch &a, const Branch &b)
      {
         return !(a == b);
      }
   };

   /// Node for each branch level
   struct Node
   {
      Node(int id) : ID(id), m_count(0), m_level(-1) {}

      bool IsInternalNode() const
      {
         return (m_level > 0);
      } // Not a leaf, but a internal node
      bool IsLeaf() const { return (m_level == 0); } // A leaf, contains data

      int                          ID;       ///< ID
      int                          m_count;  ///< Count
      int                          m_level;  ///< Leaf is zero, others positive
      std::array<Branch, MAXNODES> m_branch; ///< Branch

      friend bool operator==(const Node &a, const Node &b)
      {
         if (a.m_count != b.m_count)
            return false;
         bool match{true};
         for (int i = 0; i < a.m_count; ++i)
         {
            match = match && (a.m_branch[i] == b.m_branch[i]);
         }

         return (a.ID == b.ID) && (a.m_level == b.m_level) && match;
      }
      friend bool operator!=(const Node &a, const Node &b) { return !(a == b); }
   };

   int                           node_ID; ///< Global node Id
   int                           root_ID; ///< Root node Id
   std::unordered_map<int, Node> Nodes;   ///< node list
   mutable std::vector<DATATYPE> results;
   ELEMTYPEREAL
   m_unitSphereVolume; ///< Unit sphere constant for required number of dimensions

   /// Variables for finding a split partition
   struct PartitionVars
   {
      static constexpr int          NOT_TAKEN = -1; // indicates that position
      std::array<int, MAXNODES + 1> m_partition;
      int                           m_total;
      int                           m_minFill;
      std::array<int, 2>            m_count;
      std::array<Rect, 2>           m_cover;
      std::array<ELEMTYPEREAL, 2>   m_area;

      std::array<Branch, MAXNODES + 1> m_branchBuf;
      int                              m_branchCount;
      Rect                             m_coverSplit;
      ELEMTYPEREAL                     m_coverSplitArea;

      PartitionVars()
          : m_total(0), m_minFill(), m_count({0, 0}), m_area({0, 0}),
            m_branchCount(0), m_coverSplitArea(0)
      {
         m_partition.fill(NOT_TAKEN);
      }
   };

   //Needed for removing entries
   struct parent_t
   {
      int parent_id;
      int index;
   };
   std::unordered_map<int, parent_t> child_parent;

   friend bool operator==(const RTree &a, const RTree &b)
   {
      return (a.Nodes == b.Nodes) && (a.root_ID == b.root_ID);
   }

   //Allocation/Deallocation
   int  AllocNode();
   void FreeNode(int node_id);

   //Helper functions
   void create_root();

   //Insertion functions
   bool InsertRectIterate(const Branch &a_branch, int node_id, int *newNode,
                          int a_level);
   bool InsertRect(const Branch &a_branch, int a_level);
   Rect NodeCover(int node_id) const;
   bool AddBranch(const Branch &a_branch, int node_id, int *newNode);
   void ChoosePartition(PartitionVars &a_parVars, int a_minFill);
   void LoadNodes(int node_idA, int node_idB, const PartitionVars &a_parVars);
   void PickSeeds(PartitionVars &a_parVars);
   void Classify(int a_index, int a_group, PartitionVars &a_parVars);
   int  PickBranch(const Rect &a_rect, int node_id);
   void SplitNode(int node_id, const Branch &a_branch, int *a_newNode);

   //Rectangle functions
   Rect         CombineRect(const Rect &a_rectA, const Rect &a_rectB) const;
   bool         Overlap(const Rect &a_rectA, const Rect &a_rectB) const;
   ELEMTYPEREAL RectSphericalVolume(const Rect &a_rect) const;
   ELEMTYPEREAL RectVolume(const Rect &a_rect) const;
   ELEMTYPEREAL CalcRectVolume(const Rect &a_rect) const;

   //Deletion functions
   void Remove(const Rect &a_rect, const DATATYPE &a_id);
   void DisconnectBranch(int node_id, int a_index);

   //Search functions
   bool Search(const Rect &a_rect, int &a_foundCount,
               std::function<bool(const DATATYPE &)> callback) const;
   bool Search(const Rect &a_rect, int &a_foundCount, int max_count) const;

   struct file_header_t
   {
      int dataFileId;
      int dataSize;
      int dataNumDims;
      int dataElemSize;
      int dataElemRealSize;
      int dataMaxNodes;
      int dataMinNodes;
   };
   friend bool operator==(const file_header_t &a, const file_header_t &b)
   {
      bool val = (a.dataFileId == b.dataFileId) && (a.dataSize == b.dataSize) &&
                 (a.dataNumDims == b.dataNumDims) &&
                 (a.dataElemSize == b.dataElemSize) &&
                 (a.dataElemRealSize == b.dataElemRealSize) &&
                 (a.dataMaxNodes == b.dataMaxNodes) &&
                 (a.dataMinNodes == b.dataMinNodes);
      return val;
   };

   const file_header_t file_header{('R' << 0) | ('T' << 8) | ('R' << 16) |
                                       ('E' << 24),
                                   sizeof(DATATYPE),
                                   NUMDIMS,
                                   sizeof(ELEMTYPE),
                                   sizeof(ELEMTYPEREAL),
                                   TMAXNODES,
                                   TMINNODES};

   friend std::ostream &operator<<(std::ostream &os, const Rect &b)
   {
      for (int i = 0; i < NUMDIMS; ++i)
      {
         os << "  " << b.m_min[i] << "  " << b.m_max[i] << '\n';
      }
      return os;
   }
   friend std::istream &operator>>(std::istream &is, Rect &b)
   {
      for (int i = 0; i < NUMDIMS; ++i)
      {
         is >> b.m_min[i] >> b.m_max[i];
      }
      return is;
   }

   friend std::ostream &operator<<(std::ostream &os, const Branch &b)
   {
      os << b.child_ID << " " << b.m_data << '\n';
      os << b.m_rect;
      return os;
   }
   friend std::istream &operator>>(std::istream &is, Branch &b)
   {
      is >> b.child_ID >> b.m_data;
      is >> b.m_rect;
      return is;
   }

   friend std::ostream &operator<<(std::ostream &os, const Node &n)
   {
      os << n.ID << " " << n.m_count << " " << n.m_level << '\n';
      for (int i = 0; i < n.m_count; ++i)
      {
         os << n.m_branch[i];
      }
      os << std::endl;
      return os;
   }

   friend std::istream &operator>>(std::istream &is, Node &n)
   {
      is >> n.ID >> n.m_count >> n.m_level;
      for (int i = 0; i < n.m_count; ++i)
      {
         Branch branch;
         is >> branch;
         n.m_branch[i] = branch;
      }
      return is;
   }

   friend std::ostream &operator<<(std::ostream &os, const file_header_t &h)
   {
      os << h.dataFileId << '\n'
         << h.dataSize << '\n'
         << h.dataNumDims << '\n'
         << h.dataElemSize << '\n'
         << h.dataElemRealSize << '\n'
         << h.dataMaxNodes << '\n'
         << h.dataMinNodes << '\n';
      return os;
   }
   friend std::istream &operator>>(std::istream &os, file_header_t &h)
   {
      os >> h.dataFileId >> h.dataSize >> h.dataNumDims >> h.dataElemSize >>
          h.dataElemRealSize >> h.dataMaxNodes >> h.dataMinNodes;
      return os;
   }

   friend std::ostream &operator<<(std::ostream &os, const RTree &tree)
   {
      os << tree.file_header;
      os << tree.root_ID << '\n';
      for (const auto &node : tree.Nodes)
      {
         os << node.first << '\n';
         os << node.second;
      }
      return os;
   }

   friend std::istream &operator>>(std::istream &is, RTree &tree)
   {
      RTree::file_header_t header;
      is >> header;
      if (header == tree.file_header)
      {
         int root_ID{-1};
         is >> root_ID;
         RTree::Node                          node(-1);
         std::unordered_map<int, RTree::Node> Nodes;
         int                                  key;
         while (is >> key)
         {
            is >> node;
            Nodes.emplace(key, node);
         }
         std::swap(tree.Nodes, Nodes);
         std::swap(tree.root_ID, root_ID);
      }
      return is;
   }
};

RTREE_TEMPLATE
RTREE_QUAL::RTree()
{
   static_assert(MAXNODES > MINNODES);
   static_assert(MINNODES > 0);
   static_assert(NUMDIMS > 0);

   results.resize(100);
   create_root();

   constexpr double n               = static_cast<double>(NUMDIMS) / 2.;
   constexpr double unit_sphere_vol = pow(Pi, n) / tgamma(n + 1.);
   m_unitSphereVolume               = (ELEMTYPEREAL)unit_sphere_vol;
}

RTREE_TEMPLATE
void RTREE_QUAL::create_root()
{
   node_ID      = 0;
   auto id      = AllocNode();
   root_ID      = id;
   Node &root   = Nodes.at(root_ID);
   root.m_level = 0;
}

RTREE_TEMPLATE
void RTREE_QUAL::Insert(const std::array<ELEMTYPE, NUMDIMS> &a_min,
                        const std::array<ELEMTYPE, NUMDIMS> &a_max,
                        const DATATYPE &                     a_dataId)
{

   Branch branch;
   branch.m_data   = a_dataId;
   branch.child_ID = -1;

   branch.m_rect = Rect(a_min, a_max);

   InsertRect(branch, 0);
}

RTREE_TEMPLATE
void RTREE_QUAL::Remove(const std::array<ELEMTYPE, NUMDIMS> &a_min,
                        const std::array<ELEMTYPE, NUMDIMS> &a_max,
                        const DATATYPE &                     a_dataId)
{

   Rect rect(a_min, a_max);
   Remove(rect, a_dataId);
}

RTREE_TEMPLATE
int RTREE_QUAL::Search(const std::array<ELEMTYPE, NUMDIMS> & a_min,
                       const std::array<ELEMTYPE, NUMDIMS> & a_max,
                       std::function<bool(const DATATYPE &)> callback) const
{

   Rect rect(a_min, a_max);

   int foundCount = 0;
   Search(rect, foundCount, callback);

   return foundCount;
}

RTREE_TEMPLATE
std::tuple<typename RTREE_QUAL::result_iterator,
           typename RTREE_QUAL::result_iterator>
RTREE_QUAL::Search(const std::array<ELEMTYPE, NUMDIMS> &a_min,
                   const std::array<ELEMTYPE, NUMDIMS> &a_max,
                   int                                  max_count) const
{

   Rect rect(a_min, a_max);

   int foundCount = 0;
   Search(rect, foundCount, max_count);
   return std::make_tuple(results.begin(), results.begin() + foundCount);
}

RTREE_TEMPLATE
size_t RTREE_QUAL::Count() const
{
   size_t mycount = 0;
   for (const auto &[key, node] : Nodes)
   {
      if (node.IsLeaf())
      {
         mycount += node.m_count;
      }
   }

   return mycount;
}

// Delete all existing nodes
RTREE_TEMPLATE
void RTREE_QUAL::RemoveAll()
{

   Nodes.clear();
   child_parent.clear();
   create_root();
}

RTREE_TEMPLATE
int RTREE_QUAL::AllocNode()
{
   auto index = node_ID;
   Nodes.emplace(index, Node{index});
   ++node_ID;
   return index;
}

RTREE_TEMPLATE
void RTREE_QUAL::FreeNode(int node_id) { Nodes.erase(node_id); }

// Insert a data rectangle into an index structure.
// InsertRect provides for splitting the root;
// returns true if root was split, false if it was not.
// The level argument specifies the number of steps up from the leaf
// level to insert; e.g. a data rectangle goes in at level = 0.
//
RTREE_TEMPLATE
bool RTREE_QUAL::InsertRect(const Branch &a_branch, int a_level)
{

#ifdef _DEBUG
   for (int index = 0; index < NUMDIMS; ++index)
   {
      ASSERT(a_branch.m_rect.m_min[index] <= a_branch.m_rect.m_max[index]);
   }
#endif //_DEBUG
   auto &root = Nodes.at(root_ID);
   int   newNode;

   if (InsertRectIterate(a_branch, root_ID, &newNode, a_level)) // Root split
   {
      auto  id        = AllocNode();
      auto &newRoot   = Nodes.at(id);
      newRoot.m_level = root.m_level + 1;
      root_ID         = newRoot.ID;
      Branch branch;

      // add old root node as a child of the new root
      branch.m_rect   = NodeCover(root.ID);
      branch.child_ID = root.ID;
      AddBranch(branch, newRoot.ID, nullptr);
      // add the split node as a child of the new root
      branch.m_rect   = NodeCover(newNode);
      branch.child_ID = newNode;
      AddBranch(branch, newRoot.ID, nullptr);
      return true;
   }

   return false;
}

// Inserts a new data rectangle into the index structure.
RTREE_TEMPLATE
bool RTREE_QUAL::InsertRectIterate(const Branch &a_branch, int node_id,
                                   int *newNode, int a_level)
{
   struct path_t
   {
      int node;
      int branch;
   };

   std::stack<path_t> path;
   int                test_id = node_id;
   auto               level   = Nodes.at(test_id).m_level;
   int                index   = PickBranch(a_branch.m_rect, test_id);
   while (level > a_level)
   {
      path.push({test_id, index});
      test_id = Nodes.at(test_id).m_branch[index].child_ID;
      level   = Nodes.at(test_id).m_level;
      index   = PickBranch(a_branch.m_rect, test_id);
   }

   int  otherNode;
   bool childWasSplit = AddBranch(a_branch, test_id, &otherNode);

   while (!path.empty())
   {
      auto [id, ind] = path.top();
      auto &node     = Nodes.at(id);
      if (!childWasSplit)
      {
         // Child was not split. Merge the bounding box of the new record with the
         // existing bounding box
         node.m_branch[ind].m_rect =
             CombineRect(a_branch.m_rect, (node.m_branch[ind].m_rect));
         childWasSplit = false;
      }
      else
      {
         // Child was split. The old branches are now re-partitioned to two nodes
         // so we have to re-calculate the bounding boxes of each node
         node.m_branch[ind].m_rect = NodeCover(node.m_branch[ind].child_ID);
         Branch branch;
         branch.child_ID = otherNode;
         branch.m_rect   = NodeCover(otherNode);
         // The old node is already a child of a_node. Now add the newly-created
         // node to a_node as well. a_node might be split because of that.
         childWasSplit = AddBranch(branch, id, &otherNode);
      }

      path.pop();
   }

   *newNode = otherNode;
   return childWasSplit;
}

// Find the smallest rectangle that includes all rectangles in branches of a node.
RTREE_TEMPLATE
typename RTREE_QUAL::Rect RTREE_QUAL::NodeCover(int node_id) const
{
   const auto &a_node = Nodes.at(node_id);
   Rect        rect   = a_node.m_branch[0].m_rect;
   for (int index = 1; index < a_node.m_count; ++index)
   {
      rect = CombineRect(rect, (a_node.m_branch[index].m_rect));
   }

   return rect;
}

// Add a branch to a node.  Split the node if necessary.
// Returns false if node not split.  Old node updated.
// Returns true if node split, sets *new_node to address of new node.
// Old node updated, becomes one of two.
RTREE_TEMPLATE
bool RTREE_QUAL::AddBranch(const Branch &a_branch, int node_id, int *newNode)
{
   auto &a_node = Nodes.at(node_id);

   if (a_node.m_count < MAXNODES) // Split won't be necessary
   {
      if (a_branch.child_ID != -1 && a_node.IsInternalNode())
      {
         child_parent.insert_or_assign(a_branch.child_ID,
                                       parent_t{node_id, a_node.m_count});
      }
      a_node.m_branch[a_node.m_count] = a_branch;
      ++a_node.m_count;

      return false;
   }
   else
   {
      SplitNode(a_node.ID, a_branch, newNode);
      return true;
   }
}

// Disconnect a dependent node.
// Caller must return (or stop using iteration index) after this as count has changed
RTREE_TEMPLATE
void RTREE_QUAL::DisconnectBranch(int node_id, int a_index)
{
   auto &a_node = Nodes.at(node_id);
   // Remove element by swapping with the last element to prevent gaps in array
   a_node.m_branch[a_index] = a_node.m_branch[a_node.m_count - 1];
   --a_node.m_count;
   if (a_node.m_branch[a_index].child_ID != -1)
   {
      child_parent.insert_or_assign(a_node.m_branch[a_index].child_ID,
                                    parent_t{node_id, a_index});
   }
}

// Pick a branch.  Pick the one that will need the smallest increase
// in area to accomodate the new rectangle.  This will result in the
// least total area for the covering rectangles in the current node.
// In case of a tie, pick the one which was smaller before, to get
// the best resolution when searching.
RTREE_TEMPLATE
int RTREE_QUAL::PickBranch(const Rect &a_rect, int node_id)
{
   auto &a_node = Nodes.at(node_id);

   bool         firstTime = true;
   ELEMTYPEREAL increase;
   ELEMTYPEREAL bestIncr = (ELEMTYPEREAL)-1;
   ELEMTYPEREAL area;
   ELEMTYPEREAL bestArea;
   int          best = 0;
   Rect         tempRect;

   for (int index = 0; index < a_node.m_count; ++index)
   {
      Rect &curRect = a_node.m_branch[index].m_rect;
      area          = CalcRectVolume(curRect);
      tempRect      = CombineRect(a_rect, curRect);
      increase      = CalcRectVolume(tempRect) - area;
      if ((increase < bestIncr) || firstTime)
      {
         best      = index;
         bestArea  = area;
         bestIncr  = increase;
         firstTime = false;
      }
      else if ((increase == bestIncr) && (area < bestArea))
      {
         best     = index;
         bestArea = area;
         bestIncr = increase;
      }
   }
   return best;
}

// Combine two rectangles into larger one containing both
RTREE_TEMPLATE
typename RTREE_QUAL::Rect RTREE_QUAL::CombineRect(const Rect &a_rectA,
                                                  const Rect &a_rectB) const
{
   Rect newRect;

   for (int index = 0; index < NUMDIMS; ++index)
   {
      newRect.m_min[index] =
          std::min(a_rectA.m_min[index], a_rectB.m_min[index]);
      newRect.m_max[index] =
          std::max(a_rectA.m_max[index], a_rectB.m_max[index]);
   }

   return newRect;
}

// Split a node.
// Divides the nodes branches and the extra one between two nodes.
// Old node is one of the new ones, and one really new one is created.
// Tries more than one method for choosing a partition, uses best result.
RTREE_TEMPLATE
void RTREE_QUAL::SplitNode(int node_id, const Branch &a_branch, int *newNode)
{
   auto &a_node = Nodes.at(node_id);

   PartitionVars localVars;

   // Load all the branches into a buffer, initialize old node
   // Load the branch buffer
   for (int index = 0; index < MAXNODES; ++index)
   {
      localVars.m_branchBuf[index] = a_node.m_branch[index];
   }
   localVars.m_branchBuf[MAXNODES] = a_branch;
   localVars.m_branchCount         = MAXNODES + 1;
   localVars.m_total               = localVars.m_branchCount;

   // Calculate rect containing all in the set
   localVars.m_coverSplit = localVars.m_branchBuf[0].m_rect;
   for (int index = 1; index < MAXNODES + 1; ++index)
   {
      localVars.m_coverSplit = CombineRect(localVars.m_coverSplit,
                                           localVars.m_branchBuf[index].m_rect);
   }
   localVars.m_coverSplitArea = CalcRectVolume(localVars.m_coverSplit);

   // Find partition
   ChoosePartition(localVars, MINNODES);

   // Create a new node to hold (about) half of the branches
   auto  id        = AllocNode();
   auto &newnode   = Nodes.at(id);
   *newNode        = id;
   newnode.m_level = a_node.m_level;

   // Put branches from buffer into 2 nodes according to the chosen partition
   a_node.m_count = 0;
   LoadNodes(a_node.ID, id, localVars);

   ASSERT((a_node.m_count + (newnode).m_count) == localVars.m_total);
}

// Calculate the n-dimensional volume of a rectangle
RTREE_TEMPLATE
ELEMTYPEREAL RTREE_QUAL::RectVolume(const Rect &a_rect) const
{

   ELEMTYPEREAL volume = (ELEMTYPEREAL)1;

   for (int index = 0; index < NUMDIMS; ++index)
   {
      volume *= a_rect.m_max[index] - a_rect.m_min[index];
   }

   ASSERT(volume >= (ELEMTYPEREAL)0);

   return volume;
}

// The exact volume of the bounding sphere for the given Rect
RTREE_TEMPLATE
ELEMTYPEREAL RTREE_QUAL::RectSphericalVolume(const Rect &a_rect) const
{

   ELEMTYPEREAL sumOfSquares = (ELEMTYPEREAL)0;
   ELEMTYPEREAL radius;

   for (int index = 0; index < NUMDIMS; ++index)
   {
      ELEMTYPEREAL halfExtent = ((ELEMTYPEREAL)a_rect.m_max[index] -
                                 (ELEMTYPEREAL)a_rect.m_min[index]) *
                                (ELEMTYPEREAL)0.5;
      sumOfSquares += halfExtent * halfExtent;
   }

   radius = (ELEMTYPEREAL)sqrt(sumOfSquares);

   if (NUMDIMS == 3)
   {
      return (radius * radius * radius * m_unitSphereVolume);
   }
   else if (NUMDIMS == 2)
   {
      return (radius * radius * m_unitSphereVolume);
   }
   else
   {
      return (ELEMTYPEREAL)(pow(radius, NUMDIMS) * m_unitSphereVolume);
   }
}

// Use one of the methods to calculate retangle volume
RTREE_TEMPLATE
ELEMTYPEREAL RTREE_QUAL::CalcRectVolume(const Rect &a_rect) const
{
#ifdef RTREE_USE_SPHERICAL_VOLUME
   return RectSphericalVolume(a_rect); // Slower but helps certain merge cases
#else                                  // RTREE_USE_SPHERICAL_VOLUME
   return RectVolume(a_rect); // Faster but can cause poor merges
#endif                                 // RTREE_USE_SPHERICAL_VOLUME
}

// Method #0 for choosing a partition:
// As the seeds for the two groups, pick the two rects that would waste the
// most area if covered by a single rectangle, i.e. evidently the worst pair
// to have in the same group.
// Of the remaining, one at a time is chosen to be put in one of the two groups.
// The one chosen is the one with the greatest difference in area expansion
// depending on which group - the rect most strongly attracted to one group
// and repelled from the other.
// If one group gets too full (more would force other group to violate min
// fill requirement) then other group gets the rest.
// These last are the ones that can go in either group most easily.
RTREE_TEMPLATE
void RTREE_QUAL::ChoosePartition(PartitionVars &a_parVars, int a_minFill)
{

   ELEMTYPEREAL biggestDiff;
   int          group, chosen = 0, betterGroup = 0;

   a_parVars.m_minFill = a_minFill;

   PickSeeds(a_parVars);

   while (((a_parVars.m_count[0] + a_parVars.m_count[1]) < a_parVars.m_total) &&
          (a_parVars.m_count[0] < (a_parVars.m_total - a_parVars.m_minFill)) &&
          (a_parVars.m_count[1] < (a_parVars.m_total - a_parVars.m_minFill)))
   {
      biggestDiff = (ELEMTYPEREAL)-1;
      for (int index = 0; index < a_parVars.m_total; ++index)
      {
         if (PartitionVars::NOT_TAKEN == a_parVars.m_partition[index])
         {
            Rect &       curRect = a_parVars.m_branchBuf[index].m_rect;
            Rect         rect0   = CombineRect(curRect, a_parVars.m_cover[0]);
            Rect         rect1   = CombineRect(curRect, a_parVars.m_cover[1]);
            ELEMTYPEREAL growth0 = CalcRectVolume(rect0) - a_parVars.m_area[0];
            ELEMTYPEREAL growth1 = CalcRectVolume(rect1) - a_parVars.m_area[1];
            ELEMTYPEREAL diff    = growth1 - growth0;
            if (diff >= 0)
            {
               group = 0;
            }
            else
            {
               group = 1;
               diff  = -diff;
            }

            if (diff > biggestDiff)
            {
               biggestDiff = diff;
               chosen      = index;
               betterGroup = group;
            }
            else if ((diff == biggestDiff) && (a_parVars.m_count[group] <
                                               a_parVars.m_count[betterGroup]))
            {
               chosen      = index;
               betterGroup = group;
            }
         }
      }
      Classify(chosen, betterGroup, a_parVars);
   }

   // If one group too full, put remaining rects in the other
   if ((a_parVars.m_count[0] + a_parVars.m_count[1]) < a_parVars.m_total)
   {
      if (a_parVars.m_count[0] >= a_parVars.m_total - a_parVars.m_minFill)
      {
         group = 1;
      }
      else
      {
         group = 0;
      }
      for (int index = 0; index < a_parVars.m_total; ++index)
      {
         if (PartitionVars::NOT_TAKEN == a_parVars.m_partition[index])
         {
            Classify(index, group, a_parVars);
         }
      }
   }

   ASSERT((a_parVars.m_count[0] + a_parVars.m_count[1]) == a_parVars.m_total);
   ASSERT((a_parVars.m_count[0] >= a_parVars.m_minFill) &&
          (a_parVars.m_count[1] >= a_parVars.m_minFill));
}

// Copy branches from the buffer into two nodes according to the partition.
RTREE_TEMPLATE
void RTREE_QUAL::LoadNodes(int node_idA, int node_idB,
                           const PartitionVars &a_parVars)
{
   for (int index = 0; index < a_parVars.m_total; ++index)
   {
      ASSERT(a_parVars.m_partition[index] == 0 ||
             a_parVars.m_partition[index] == 1);

      int  targetNodeIndex = a_parVars.m_partition[index];
      auto id              = targetNodeIndex == 0 ? node_idA : node_idB;
      // It is assured that AddBranch here will not cause a node split.
      bool nodeWasSplit = AddBranch(a_parVars.m_branchBuf[index], id, NULL);
      ASSERT(!nodeWasSplit);
   }
}

RTREE_TEMPLATE
void RTREE_QUAL::PickSeeds(PartitionVars &a_parVars)
{
   int          seed0 = 0, seed1 = 0;
   ELEMTYPEREAL worst, waste;
   ELEMTYPEREAL area[MAXNODES + 1];

   for (int index = 0; index < a_parVars.m_total; ++index)
   {
      area[index] = CalcRectVolume(a_parVars.m_branchBuf[index].m_rect);
   }

   worst = -a_parVars.m_coverSplitArea - 1;
   for (int indexA = 0; indexA < a_parVars.m_total - 1; ++indexA)
   {
      for (int indexB = indexA + 1; indexB < a_parVars.m_total; ++indexB)
      {
         Rect oneRect = CombineRect(a_parVars.m_branchBuf[indexA].m_rect,
                                    a_parVars.m_branchBuf[indexB].m_rect);
         waste        = CalcRectVolume(oneRect) - area[indexA] - area[indexB];
         if (waste > worst)
         {
            worst = waste;
            seed0 = indexA;
            seed1 = indexB;
         }
      }
   }

   Classify(seed0, 0, a_parVars);
   Classify(seed1, 1, a_parVars);
}

// Put a branch in one of the groups.
RTREE_TEMPLATE
void RTREE_QUAL::Classify(int a_index, int a_group, PartitionVars &a_parVars)
{
   ASSERT(PartitionVars::NOT_TAKEN == a_parVars.m_partition[a_index]);

   a_parVars.m_partition[a_index] = a_group;

   // Calculate combined rect
   if (a_parVars.m_count[a_group] == 0)
   {
      a_parVars.m_cover[a_group] = a_parVars.m_branchBuf[a_index].m_rect;
   }
   else
   {
      a_parVars.m_cover[a_group] = CombineRect(
          a_parVars.m_branchBuf[a_index].m_rect, a_parVars.m_cover[a_group]);
   }

   // Calculate volume of combined rect
   a_parVars.m_area[a_group] = CalcRectVolume(a_parVars.m_cover[a_group]);

   ++a_parVars.m_count[a_group];
}

RTREE_TEMPLATE
void RTREE_QUAL::Remove(const Rect &a_rect, const DATATYPE &a_id)
{
   std::forward_list<int> node_list;

   for (const auto &[key, node] : Nodes)
   {
      if (!node.IsInternalNode())
      {
         std::stack<int> indices;
         for (int index = 0; index < node.m_count; ++index)
         {
            if (Overlap(a_rect, node.m_branch[index].m_rect))
            {
               if (node.m_branch[index].m_data == a_id)
               {
                  indices.push(index);
               }
            }
         }
         //Found items, now cleanup
         if (!indices.empty())
         {

            while (!indices.empty())
            {
               auto ind = indices.top();
               DisconnectBranch(node.ID, ind);
               indices.pop();
            }
            auto [parent_id, index] = child_parent.at(node.ID);
            auto child_id           = node.ID;

            while (true)
            {
               auto &parent = Nodes.at(parent_id);
               auto &child  = Nodes.at(child_id);
               if (child.m_count >= MINNODES)
               {
                  //resize parent
                  parent.m_branch[index].m_rect = NodeCover(child_id);
               }
               else
               {
                  //eliminate node
                  node_list.push_front(child_id);
                  DisconnectBranch(parent_id, index);
               }
               if (parent_id == root_ID)
                  break;
               auto [new_parent_id, new_index] = child_parent.at(parent_id);
               child_id                        = parent_id;
               parent_id                       = new_parent_id;
               index                           = new_index;
            }
         }
      }
   }

   while (!node_list.empty())
   {
      auto  id       = node_list.front();
      auto &tempNode = Nodes.at(id);
      for (int index = 0; index < tempNode.m_count; ++index)
      {
         // TODO go over this code. should I use (tempNode->m_level - 1)?
         InsertRect(tempNode.m_branch[index], tempNode.m_level);
      }
      FreeNode(id);
      child_parent.erase(id);
      node_list.pop_front();
   }

   // Check for redundant root (not leaf, 1 child) and eliminate TODO replace
   // if with while? In case there is a whole branch of redundant roots...
   auto &root = Nodes.at(root_ID);
   if (root.m_count == 1 && root.IsInternalNode())
   {
      auto new_root_ID = root.m_branch[0].child_ID;
      FreeNode(root_ID);
      child_parent.erase(root_ID);
      root_ID = new_root_ID;
   }
}

// Decide whether two rectangles overlap.
RTREE_TEMPLATE
bool RTREE_QUAL::Overlap(const Rect &a_rectA, const Rect &a_rectB) const
{
   for (int index = 0; index < NUMDIMS; ++index)
   {
      if (a_rectA.m_min[index] > a_rectB.m_max[index] ||
          a_rectB.m_min[index] > a_rectA.m_max[index])
      {
         return false;
      }
   }
   return true;
}

// Search in an index tree or subtree for all data retangles that overlap the argument rectangle.
RTREE_TEMPLATE
bool RTREE_QUAL::Search(const Rect &a_rect, int &a_foundCount,
                        std::function<bool(const DATATYPE &)> callback) const
{

   for (const auto &[key, node] : Nodes)
   {
      if (!node.IsInternalNode())
      {
         for (int index = 0; index < node.m_count; ++index)
         {
            if (Overlap(a_rect, node.m_branch[index].m_rect))
            {
               const DATATYPE &id = node.m_branch[index].m_data;
               ++a_foundCount;

               if (callback && !callback(id))
               {
                  return false; // Don't continue searching
               }
            }
         }
      }
   }

   return true; // Continue searching*/
}

RTREE_TEMPLATE
bool RTREE_QUAL::Search(const Rect &a_rect, int &a_foundCount,
                        int max_count) const
{
   for (const auto &[key, node] : Nodes)
   {
      if (!node.IsInternalNode())
      {

         for (int index = 0; index < node.m_count; ++index)
         {
            if (Overlap(a_rect, node.m_branch[index].m_rect))
            {
               DATATYPE id = node.m_branch[index].m_data;
               if (a_foundCount < (int)results.size())
               {
                  results[a_foundCount] = id;
               }
               else
               {
                  results.resize(2 * results.size());
                  results[a_foundCount] = id;
               }
               ++a_foundCount;
               if (a_foundCount == max_count)
               {
                  return false;
               }
            }
         }
      }
   }

   return true;
}

#undef RTREE_TEMPLATE
#undef RTREE_QUAL

#endif //RTREE_H
