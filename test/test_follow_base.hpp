// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// gtest
#include <gtest/gtest.h>

// std
#include <fstream>
#include <string>
#include <cmath>

#include "../test/test_helper.h"

class TestFollow : public ::testing::Test
{
public:
  TestFollow()
  : input_data(),
    output_data(),
    path(std::string(TEST_DIR))
  {
  }

  virtual ~TestFollow() = default;
  virtual void readInputData() = 0;
  virtual void readOutputData() = 0;
  virtual void writeEstimatedData() = 0;
  virtual void checkData() = 0;
  virtual void update() = 0;


  void openFiles(
    std::string input_filename,
    std::string estimate_filename,
    std::ios_base::openmode mode = std::ios_base::in)
  {
    input_data.open(path + "/" + input_filename);
    ASSERT_TRUE(input_data.is_open());
    output_data.open(path + "/" + estimate_filename, mode);
    ASSERT_TRUE(input_data.is_open());
  }

  void generateEstimatedDataFile(
    std::string input_filename,
    std::string estimate_filename)
  {
    openFiles(input_filename, estimate_filename, std::fstream::out);

    while (!input_data.eof()) {
      readInputData();
      update();
      writeEstimatedData();
    }
  }

  void check()
  {
    while (!input_data.eof()) {
      readInputData();
      readOutputData();
      update();
      checkData();
    }
  }


  std::ifstream input_data;
  std::fstream output_data;
  std::string path;
};
