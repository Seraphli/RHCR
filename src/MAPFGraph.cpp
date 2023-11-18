#include "MAPFGraph.h"
#include <boost/tokenizer.hpp>
#include <sstream> // std::istringstream

bool MAPFGraph::load_map(string fname) {
  using namespace boost;
  string line;
  std::ifstream myfile((fname).c_str());
  if (!myfile.is_open()) {
    std::cout << "Map file " << fname << " does not exist. " << std::endl;
    return false;
  }

  if (std_out)
    std::cout << "*** Loading map ***" << std::endl;
  clock_t t = std::clock();
  std::size_t pos = fname.rfind('.'); // position of the file extension
  map_name = fname.substr(0, pos);    // get the name without extension

  tokenizer<char_separator<char>>::iterator beg;
  getline(myfile, line); // skip word "type:*"
  char_separator<char> sep(" ");
  getline(myfile, line);
  tokenizer<char_separator<char>> tok(line, sep);
  beg = tok.begin();
  beg++;
  this->rows = atoi((*beg).c_str()); // read number of rows
  getline(myfile, line);
  tokenizer<char_separator<char>> tok2(line, sep);
  beg = tok2.begin();
  beg++;
  this->cols = atoi((*beg).c_str()); // read number of cols
  move[0] = 1;
  move[1] = -cols;
  move[2] = -1;
  move[3] = cols;
  getline(myfile, line); // skip word "map"

  // read tyeps and edge weights
  this->types.resize(rows * cols);
  this->weights.resize(rows * cols);
  // read map (and start/goal locations)
  for (int i = 0; i < rows; i++) {
    getline(myfile, line);
    for (int j = 0; j < cols; j++) {
      int id = cols * i + j;
      weights[id].resize(5, WEIGHT_MAX);
      if (line[j] == '.') {
        this->types[id] = "Travel";
      } else {
        this->types[id] = "Obstacle";
      }
    }
  }

  for (int i = 0; i < cols * rows; i++) {
    if (types[i] == "Obstacle") {
      continue;
    }
    for (int dir = 0; dir < 4; dir++) {
      if (0 <= i + move[dir] && i + move[dir] < cols * rows &&
          get_Manhattan_distance(i, i + move[dir]) <= 1 &&
          types[i + move[dir]] != "Obstacle")
        weights[i][dir] = 1;
    }
  }

  myfile.close();
  double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
  if (std_out) {
    std::cout << "Map size: " << rows << "x" << cols << std::endl;
    std::cout << "Done! (" << runtime << " s)" << std::endl;
  }
  return true;
}

void MAPFGraph::preprocessing(bool consider_rotation) {
  this->consider_rotation = consider_rotation;
}
