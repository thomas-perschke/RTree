//
// TestBadData.cpp
//

#include <iostream>
#include <fstream>
#include <vector>
#include <iostream>
#include "RTree.h"

typedef int       ValueType;
typedef long long CoordType;

struct Rect
{
   Rect() {}

   Rect(CoordType a_minX, CoordType a_minY, CoordType a_maxX, CoordType a_maxY)
   {
      min[0] = a_minX;
      min[1] = a_minY;

      max[0] = a_maxX;
      max[1] = a_maxY;
   }

   std::array<CoordType, 2> min;
   std::array<CoordType, 2> max;
};

bool MySearchCallback(ValueType id)
{
   std::cout << "Hit data rect " << id << "\n";
   return true; // keep going
}

int main(int argc, char *argv[])
{
   if (argc < 2)
   {
      std::cout << "Usage: " << argv[0] << " inFile\n";
      return -1;
   }

   typedef std::vector<Rect> RectVector;
   RectVector                rectVector;

   // read the data
   {
      std::ifstream inFile(argv[1]);
      CoordType     xmin, ymin, xmax, ymax;
      while (inFile >> xmin >> ymin >> xmax >> ymax)
      {
         // security and robustness be damned
         std::cout << xmin << " " << ymin << " " << xmax << " " << ymax << "\n";
         rectVector.push_back(Rect(xmin, ymin, xmin + xmax, ymin + ymax));
      }
   }

   typedef RTree<ValueType, CoordType, 2, float> MyTree;
   MyTree                                        tree;

   std::cout << "number of rectangles is " << rectVector.size() << "\n";

   for (size_t i = 0; i < rectVector.size(); i++)
   {
      tree.Insert(
          rectVector[i].min, rectVector[i].max,
          i); // Note, all values including zero are fine in this version
   }

   Rect search_rect(6, 4, 10, 6);
   auto nhits = tree.Search(search_rect.min, search_rect.max, MySearchCallback);

   std::cout << "Search resulted in " << nhits << " hits\n";

   // Iterator test
   int itIndex = 0;
   for (auto it = tree.begin(); it != tree.end(); ++it)
   {
      int value = *it; 

      std::array<CoordType, 2> boundsMin({0, 0});
      std::array<CoordType, 2> boundsMax({0, 0});

      it.GetBounds(boundsMin, boundsMax);
      std::cout << "it[" << itIndex++ << "] " << value << " = (" << boundsMin[0]
                << "," << boundsMin[1] << "," << boundsMax[0] << ","
                << boundsMax[1] << ")\n";
   }

   // Iterator test, alternate syntax
   itIndex = 0;
   for (const auto &val : tree)
   {
      CoordType value = val;
      std::cout << "it[" << itIndex++ << "] " << value << "\n";
   }

   return 0;
}
