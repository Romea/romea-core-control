#include <gtest/gtest.h>
#include "test_helper.h"
#include <fstream>
#include <cmath>

class TestFollow : public ::testing::Test
{
public :

  TestFollow():
    input_data(),
    output_data(),
    path(std::string(TEST_DIR))
  {

  }

  virtual ~TestFollow()=default;
  virtual void readInputData()=0;
  virtual void readOutputData()=0;
  virtual void writeEstimatedData()=0;
  virtual void checkData()=0;
  virtual void update()=0;


  void openFiles(std::string input_filename,
                 std::string estimate_filename,
                 std::ios_base::openmode mode = std::ios_base::in)
  {
    input_data.open(path +"/"+input_filename);
    ASSERT_TRUE(input_data.is_open());
    output_data.open(path+"/"+estimate_filename,mode);
    ASSERT_TRUE(input_data.is_open());
  }

  void generateEstimatedDataFile(std::string input_filename,
                                 std::string estimate_filename)
  {
    openFiles(input_filename,estimate_filename,std::fstream::out);

    while(!input_data.eof())
    {
      readInputData();
      update();
      writeEstimatedData();
    }
  }

  void check()
  {
    while(!input_data.eof())
    {
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

