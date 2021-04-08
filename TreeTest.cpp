#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <array>
#include <random>

#include "RTree.h"

namespace
{
// Simplify handling of 3 dimensional coordinate
struct Vec3
{
   /// Default constructor
   Vec3() {}

   /// Construct from three elements
   Vec3(float a_x, float a_y, float a_z)
   {
      v[0] = a_x;
      v[1] = a_y;
      v[2] = a_z;
   }

   /// Add two vectors and return result
   Vec3 operator+(const Vec3 &a_other) const
   {
      return Vec3(v[0] + a_other.v[0], v[1] + a_other.v[1],
                  v[2] + a_other.v[2]);
   }

   std::array<float, 3> v; ///< 3 float components for axes or dimensions
};
/// A user type to test with, instead of a simple type such as an 'int'
struct SomeThing
{
   SomeThing() { ++s_outstandingAllocs; }
   ~SomeThing() { --s_outstandingAllocs; }

   int m_creationCounter; ///< Just a number for identifying within test program
   Vec3 m_min,
       m_max; ///< Minimal bounding rect, values must be known and constant in order to remove from RTree

   static int
       s_outstandingAllocs; ///< Count how many outstanding objects remain
};

/// Init static
int SomeThing::s_outstandingAllocs = 0;

} // namespace
TEST_CASE("RTree Test", "[Test]")
{
   typedef int ValueType;

   struct Rect
   {
      Rect() {}

      Rect(int a_minX, int a_minY, int a_maxX, int a_maxY)
      {
         min[0] = a_minX;
         min[1] = a_minY;

         max[0] = a_maxX;
         max[1] = a_maxY;
      }

      std::array<int, 2> min;
      std::array<int, 2> max;
   };

   struct Rect rects[] = {
       Rect(0, 0, 2, 2), // xmin, ymin, xmax, ymax (for 2 dimensional RTree)
       Rect(5, 5, 7, 7),
       Rect(8, 5, 9, 6),
       Rect(7, 1, 9, 2),
   };

   int nrects = sizeof(rects) / sizeof(rects[0]);

   Rect search_rect(6, 4, 10,
                    6); // search will find above rects that this one overlaps

   auto MySearchCallback = [](ValueType id) {
      std::cout << "Hit data rect " << id << "\n";
      return true; // keep going
   };

   typedef RTree<ValueType, int, 2, float> MyTree;
   MyTree                                  tree;

   std::cout << "nrects = " << nrects << "\n";

   for (int i = 0; i < nrects; i++)
   {
      tree.Insert(
          rects[i].min, rects[i].max,
          i); // Note, all values including zero are fine in this version
   }

   SECTION("Count")
   {
      auto count = tree.Count();

      REQUIRE((size_t)nrects == count);
   }

   SECTION("Search")
   {
      auto nhits =
          tree.Search(search_rect.min, search_rect.max, MySearchCallback);

      REQUIRE(nhits == 2);
   }
   SECTION("Check bounds")
   {
      int itIndex = 0;
      for (auto it = tree.begin(); it != tree.end(); ++it)
      {
         int value = *it;

         std::array<int, 2> boundsMin({0, 0});
         std::array<int, 2> boundsMax({0, 0});
         it.GetBounds(boundsMin, boundsMax);
         REQUIRE(value == itIndex);
         REQUIRE(boundsMin[0] == rects[itIndex].min[0]);
         REQUIRE(boundsMin[1] == rects[itIndex].min[1]);
         REQUIRE(boundsMax[0] == rects[itIndex].max[0]);
         REQUIRE(boundsMax[1] == rects[itIndex].max[1]);
         ++itIndex;
      }

      REQUIRE(itIndex == nrects);
   }

   SECTION("for loop")
   {
      int itIndex = 0;
      itIndex     = 0;
      for (const auto &val : tree)
      {
         REQUIRE(val == itIndex);
         ++itIndex;
      }
   }

   SECTION("copy")
   {
      MyTree copy = tree;

      REQUIRE(copy == tree);
   }

   SECTION("iterator")
   {
      for (auto &val : tree)
      {
         val = 42;
      }
      for (auto val : tree)
      {
         REQUIRE(val == 42);
      }
   }

   SECTION("serialization")
   {
      MyTree        copy;
      std::ofstream dump_tree("tree.txt");
      if (dump_tree)
      {
         dump_tree << tree;

         std::ifstream load_tree("tree.txt");
         if (load_tree)
         {
            load_tree >> copy;
         }
      }

      REQUIRE((MyTree)copy == (MyTree)tree);
   }
}

TEST_CASE("RTree Memory Test", "[MemoryTest]")
{

   auto BoxesIntersect = [](const Vec3 &a_boxMinA, const Vec3 &a_boxMaxA,
                            const Vec3 &a_boxMinB, const Vec3 &a_boxMaxB) {
      for (int axis = 0; axis < 3; ++axis)
      {
         if (a_boxMinA.v[axis] > a_boxMaxB.v[axis] ||
             a_boxMaxA.v[axis] < a_boxMinB.v[axis])
         {
            return false;
         }
      }
      return true;
   };

   const int NUM_OBJECTS =
       1000; // Number of objects in test set, must be > FRAC_OBJECTS for this test
   const int   FRAC_OBJECTS   = 4;
   const float MAX_WORLDSIZE  = 10.0f;
   const float FRAC_WORLDSIZE = MAX_WORLDSIZE / 2;

   std::random_device                    rd;
   std::mt19937                          gen(rd());
   std::uniform_real_distribution<float> dis_min(-MAX_WORLDSIZE, MAX_WORLDSIZE);
   std::uniform_real_distribution<float> dis_extent(0.f, FRAC_WORLDSIZE);

   auto min_vec = [&gen, &dis_min]() -> Vec3 {
      return Vec3(dis_min(gen), dis_min(gen), dis_min(gen));
   };

   auto extent_vec = [&gen, &dis_extent]() -> Vec3 {
      return Vec3(dis_extent(gen), dis_extent(gen), dis_extent(gen));
   };

   // typedef the RTree useage just for conveniance with iteration
   typedef RTree<SomeThing *, float, 3> SomeThingTree;

   ASSERT(NUM_OBJECTS > FRAC_OBJECTS);

   std::array<SomeThing *, NUM_OBJECTS * 2> thingArray({nullptr});

   // Create intance of RTree

   SomeThingTree tree;

   // Add some nodes
   int counter = 0;
   for (int index = 0; index < NUM_OBJECTS; ++index)
   {
      SomeThing *newThing = new SomeThing;

      newThing->m_creationCounter = counter++;
      newThing->m_min             = min_vec();
      Vec3 extent                 = extent_vec();
      newThing->m_max             = newThing->m_min + extent;

      thingArray[counter - 1] = newThing;

      tree.Insert(newThing->m_min.v, newThing->m_max.v, newThing);
   }

   REQUIRE(tree.Count() == (size_t)NUM_OBJECTS);

   int numToStep = FRAC_OBJECTS;

   // Delete some nodes
   int items_removed{0};
   for (int index = 0; index < NUM_OBJECTS; index += numToStep)
   {
      SomeThing *curThing = thingArray[index];

      if (curThing)
      {
         tree.Remove(curThing->m_min.v, curThing->m_max.v, curThing);

         delete curThing;
         thingArray[index] = NULL;
         ++items_removed;
      }
   }

   REQUIRE(tree.Count() == (size_t)(NUM_OBJECTS - items_removed));

   // Add some more nodes
   for (int index = 0; index < items_removed; ++index)
   {
      SomeThing *newThing = new SomeThing;

      newThing->m_creationCounter = counter++;
      newThing->m_min             = min_vec();
      Vec3 extent                 = extent_vec();
      newThing->m_max             = newThing->m_min + extent;

      thingArray[counter - 1] = newThing;

      tree.Insert(newThing->m_min.v, newThing->m_max.v, newThing);
   }

   REQUIRE(tree.Count() == (size_t)NUM_OBJECTS);

   Vec3 searchMin(0, 0, 0);
   Vec3 searchMax(FRAC_WORLDSIZE, FRAC_WORLDSIZE, FRAC_WORLDSIZE);

   auto [beg, end] = tree.Search(searchMin.v, searchMax.v);
   std::vector<int> found_items_internal;
   for (auto it = beg; it != end; ++it)
   {
      found_items_internal.push_back((*it)->m_creationCounter);
   }

   std::vector<int> found_items_brute;

   for (auto val : tree)
   {
      SomeThing *curThing = val;

      if (BoxesIntersect(searchMin, searchMax, curThing->m_min,
                         curThing->m_max))
      {
         found_items_brute.push_back(curThing->m_creationCounter);
      }
   }

   std::sort(found_items_internal.begin(), found_items_internal.end());
   std::sort(found_items_brute.begin(), found_items_brute.end());
   REQUIRE(std::equal(found_items_internal.begin(), found_items_internal.end(),
                      found_items_brute.begin()) == true);

   for (auto val : tree)
   {
      SomeThing *removeElem = val;
      if (removeElem)
      {
         delete removeElem;
      }
   }
   REQUIRE(SomeThing::s_outstandingAllocs == 0);
   // Remove all contents (This would have happened automatically during destructor)
   tree.RemoveAll();
   REQUIRE(tree.Count() == 0);
}
