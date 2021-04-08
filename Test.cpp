//
// Test.cpp
//
// This is a direct port of the C version of the RTree test program.
//

#include <iostream>
#include <fstream>
#include <array>
#include "RTree.h"

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

bool MySearchCallback(ValueType id)
{
   std::cout << "Hit data rect " << id << "\n";
   return true; // keep going
}

int main()
{
   typedef RTree<ValueType, int, 2, float> MyTree;
   MyTree                                  tree;

   std::cout << "nrects = " << nrects << "\n";

   for (int i = 0; i < nrects; i++)
   {
      tree.Insert(
          rects[i].min, rects[i].max,
          i); // Note, all values including zero are fine in this version
   }

   auto nhits = tree.Search(search_rect.min, search_rect.max, MySearchCallback);

   std::cout << "Search resulted in " << nhits << " hits\n";

   int itIndex = 0;
   for (auto it = tree.begin(); it != tree.end(); ++it)
   {
      int value = *it; //notree.GetAt(it);

      std::array<int, 2> boundsMin({0, 0});
      std::array<int, 2> boundsMax({0, 0});
      it.GetBounds(boundsMin, boundsMax);
      std::cout << "it[" << itIndex++ << "] " << value << " = (" << boundsMin[0]
                << "," << boundsMin[1] << "," << boundsMax[0] << ","
                << boundsMax[1] << ")\n";
   }

   itIndex = 0;
   for (const auto &val : tree)
   {
      std::cout << "it[" << itIndex++ << "] " << val << "\n";
   }

   // test copy constructor
   MyTree copy = tree;

   if(copy == tree) std::cout << "Trees match" << std::endl;
   // Iterator test
   itIndex = 0;
   for (auto it = copy.begin(); it != copy.end(); ++it)
   {
      int value = *it; //.GetAt(it);

      std::array<int, 2> boundsMin({0, 0});
      std::array<int, 2> boundsMax({0, 0});
      it.GetBounds(boundsMin, boundsMax);
      std::cout << "it[" << itIndex++ << "] " << value << " = (" << boundsMin[0]
                << "," << boundsMin[1] << "," << boundsMax[0] << ","
                << boundsMax[1] << ")\n";
   }

   itIndex = 0;
   for (auto val : copy)
   {
      std::cout << "it[" << itIndex++ << "] " << val << "\n";
   }

   for (auto &val : copy)
   {
      val = itIndex++;
   }
   itIndex = 0;
   for (auto val : copy)
   {
      std::cout << "it[" << itIndex++ << "] " << val << "\n";
   }

   std::ofstream dump_tree("tree.txt");
   if (dump_tree)
   {
      dump_tree << copy;

      copy.RemoveAll();
      std::cout << "tree count : " << copy.Count() << std::endl;
      std::ifstream load_tree("tree.txt");
      if (load_tree)
      {
         load_tree >> copy;
         itIndex = 0;
         for (auto val : copy)
         {
            std::cout << "it[" << itIndex++ << "] " << val << "\n";
         }
      }
   }
   return 0;
}
   // Output:
   //
   /*
nrects = 4
Hit data rect 1
Hit data rect 2
Search resulted in 2 hits
it[0] 0 = (0,0,2,2)
it[1] 1 = (5,5,7,7)
it[2] 2 = (8,5,9,6)
it[3] 3 = (7,1,9,2)
it[0] 0
it[1] 1
it[2] 2
it[3] 3
it[0] 0 = (0,0,2,2)
it[1] 1 = (5,5,7,7)
it[2] 2 = (8,5,9,6)
it[3] 3 = (7,1,9,2)
it[0] 0
it[1] 1
it[2] 2
it[3] 3
it[0] 4
it[1] 5
it[2] 6
it[3] 7
tree count : 0
it[0] 4
it[1] 5
it[2] 6
it[3] 7
*/

