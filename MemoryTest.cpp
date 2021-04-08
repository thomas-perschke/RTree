#include <iostream>
#include <array>
#include <random>

#include "RTree.h"

/// Simplify handling of 3 dimensional coordinate
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

static bool BoxesIntersect(const Vec3 &a_boxMinA, const Vec3 &a_boxMaxA,
                           const Vec3 &a_boxMinB, const Vec3 &a_boxMaxB)
{
   for (int axis = 0; axis < 3; ++axis)
   {
      if (a_boxMinA.v[axis] > a_boxMaxB.v[axis] ||
          a_boxMaxA.v[axis] < a_boxMinB.v[axis])
      {
         return false;
      }
   }
   return true;
}

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

/// A callback function to obtain query results in this implementation
bool QueryResultCallback(SomeThing *a_data)
{
   printf("search found %d\n", a_data->m_creationCounter);

   return true;
}

int main()
{
   const int NUM_OBJECTS =
       1000; // Number of objects in test set, must be > FRAC_OBJECTS for this test
   const int   FRAC_OBJECTS   = 4;
   const float MAX_WORLDSIZE  = 10.0f;
   const float FRAC_WORLDSIZE = MAX_WORLDSIZE / 2;

   std::random_device rd; 
   std::mt19937 gen(rd()); 
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
      std::cout << "inserting " << newThing->m_creationCounter << std::endl;
   }

   std::cout << "tree count = " << tree.Count() << std::endl;

   int numToDelete = NUM_OBJECTS / FRAC_OBJECTS;
   int numToStep   = FRAC_OBJECTS;

   // Delete some nodes
   for (int index = 0; index < NUM_OBJECTS; index += numToStep)
   {
      SomeThing *curThing = thingArray[index];

      if (curThing)
      {
         tree.Remove(curThing->m_min.v, curThing->m_max.v, curThing);
         std::cout << "removing " << curThing->m_creationCounter << std::endl;

         delete curThing;
         thingArray[index] = NULL;
      }
   }

   std::cout << "tree count = " << tree.Count() << std::endl;

   // Add some more nodes
   for (int index = 0; index < numToDelete; ++index)
   {
      SomeThing *newThing = new SomeThing;

      newThing->m_creationCounter = counter++;
      newThing->m_min             = min_vec();
      Vec3 extent                 = extent_vec();
      newThing->m_max             = newThing->m_min + extent;

      thingArray[counter - 1] = newThing;

      tree.Insert(newThing->m_min.v, newThing->m_max.v, newThing);
      std::cout << "inserting " << newThing->m_creationCounter << std::endl;
   }

   std::cout << "tree count = " << tree.Count() << std::endl;

   Vec3 searchMin(0, 0, 0);
   Vec3 searchMax(FRAC_WORLDSIZE, FRAC_WORLDSIZE, FRAC_WORLDSIZE);
   tree.Search(searchMin.v, searchMax.v, &QueryResultCallback);

   auto [beg, end] = tree.Search(searchMin.v, searchMax.v);
   for (auto it = beg; it != end; ++it)
   {
      std::cout << (*it)->m_creationCounter << " ";
   }
   std::cout << std::endl;
   // NOTE: Even better than just dumping text, it would be nice to render the
   // tree contents and search result for visualization.

   // List values.  Iterator is NOT delete safe
   // SomeThingTree::Iterator it;
   //  for( tree.GetFirst(it); !tree.IsNull(it); tree.GetNext(it) )
   for (auto val : tree)
   {
      SomeThing *curThing = val; //tree.GetAt(it);

      if (BoxesIntersect(searchMin, searchMax, curThing->m_min,
                         curThing->m_max))
      {
         std::cout << "brute found " << curThing->m_creationCounter
                   << std::endl;
      }
   }

   // Delete our nodes, NOTE, we are NOT deleting the tree nodes, just our data
   // of course the tree will now contain invalid pointers that must not be used any more.
   //  for( tree.GetFirst(it); !tree.IsNull(it); tree.GetNext(it) )
   for (auto val : tree)
   {
      SomeThing *removeElem = val; //tree.GetAt(it);
      if (removeElem)
      {
         std::cout << "deleting " << removeElem->m_creationCounter << std::endl;
         delete removeElem;
      }
   }

   // Remove all contents (This would have happened automatically during destructor)
   tree.RemoveAll();

   if (SomeThing::s_outstandingAllocs > 0)
   {
      std::cout << "Memory leak!\n"
                << "s_outstandingAllocs = " << SomeThing::s_outstandingAllocs
                << std::endl;
   }
   else
   {
      std::cout << "No memory leaks detected by app" << std::endl;
   }

   return 0;
}
