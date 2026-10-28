// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#include "apps/helpers/config/config_file_cli11_utils.h"
#include "ocudu/support/config_parsers.h"
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <unistd.h>
#include <vector>

using namespace ocudu;

namespace {

class config_file_cli11_utils_test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir = std::filesystem::path(::testing::TempDir()) /
          fmt::format("config_file_cli11_utils_test_{}_{}",
                      ::getpid(),
                      ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::create_directories(dir);

    app.config_formatter(create_yaml_config_parser());
    app.allow_config_extras(CLI::config_extras_mode::error);
    app.add_option("--value", value);
    app.add_option("--first", first);
  }

  void TearDown() override { std::filesystem::remove_all(dir); }

  /// Writes a configuration file with the given contents and returns its path.
  std::string write_file(const std::string& name, const std::string& contents)
  {
    std::filesystem::path path = dir / name;
    std::ofstream(path) << contents;
    return path.string();
  }

  /// \brief Writes the given number of configuration files and returns their paths.
  ///
  /// File \c i sets \c value to \c i. The first file also sets \c first to 1, so that dropping it can be detected.
  std::vector<std::string> write_files(unsigned nof_files)
  {
    std::vector<std::string> files;
    for (unsigned i = 0; i != nof_files; ++i) {
      std::string contents = fmt::format("value: {}\n", i);
      if (i == 0) {
        contents += "first: 1\n";
      }
      files.push_back(write_file(fmt::format("config_{}.yml", i), contents));
    }
    return files;
  }

  /// Parses the given command line arguments, the program name excluded.
  void parse(const std::vector<std::string>& args)
  {
    std::vector<const char*> argv = {"test"};
    for (const std::string& arg : args) {
      argv.push_back(arg.c_str());
    }
    app.parse(static_cast<int>(argv.size()), argv.data());
  }

  /// Returns the arguments that pass each of the given files with its own -c option.
  static std::vector<std::string> one_option_per_file(const std::vector<std::string>& files)
  {
    std::vector<std::string> args;
    for (const std::string& file : files) {
      args.emplace_back("-c");
      args.push_back(file);
    }
    return args;
  }

  std::filesystem::path dir;
  CLI::App              app{"config file option test"};
  int                   value = -1;
  int                   first = 0;
};

} // namespace

TEST_F(config_file_cli11_utils_test, later_files_and_the_command_line_take_precedence)
{
  app_helpers::add_config_file_option(app);
  std::string base_file     = write_file("base.yml", "value: 1\nfirst: 1\n");
  std::string override_file = write_file("override.yml", "value: 2\n");

  parse({"-c", base_file, "-c", override_file});
  EXPECT_EQ(value, 2);
  EXPECT_EQ(first, 1);
}

TEST_F(config_file_cli11_utils_test, the_command_line_takes_precedence_over_the_files)
{
  app_helpers::add_config_file_option(app);
  std::string base_file     = write_file("base.yml", "value: 1\nfirst: 1\n");
  std::string override_file = write_file("override.yml", "value: 2\n");

  parse({"-c", base_file, override_file, "--value", "3"});
  EXPECT_EQ(value, 3);
  EXPECT_EQ(first, 1);
}

TEST_F(config_file_cli11_utils_test, the_maximum_number_of_files_is_read)
{
  app_helpers::add_config_file_option(app);
  std::vector<std::string> files = write_files(app_helpers::MAX_NOF_CONFIG_FILES);

  parse(one_option_per_file(files));
  EXPECT_EQ(value, app_helpers::MAX_NOF_CONFIG_FILES - 1);
  EXPECT_EQ(first, 1) << "the first configuration file was not read";
}

TEST_F(config_file_cli11_utils_test, more_files_than_the_maximum_are_rejected)
{
  app_helpers::add_config_file_option(app);
  std::vector<std::string> files = write_files(app_helpers::MAX_NOF_CONFIG_FILES + 1);

  try {
    parse(one_option_per_file(files));
    FAIL() << "more configuration files than the maximum were accepted";
  } catch (const CLI::ValidationError& error) {
    EXPECT_NE(std::string(error.what())
                  .find(fmt::format("at most {} configuration files are supported, {} were given",
                                    app_helpers::MAX_NOF_CONFIG_FILES,
                                    app_helpers::MAX_NOF_CONFIG_FILES + 1)),
              std::string::npos)
        << error.what();
  }
  EXPECT_EQ(first, 0) << "a configuration file was read before the number of files was checked";
}

TEST_F(config_file_cli11_utils_test, more_files_than_the_maximum_are_rejected_in_any_form)
{
  static constexpr unsigned max_nof_files = 3;
  app_helpers::add_config_file_option(app, max_nof_files);
  std::vector<std::string> files = write_files(max_nof_files + 1);

  // One option per file.
  EXPECT_THROW(parse(one_option_per_file(files)), CLI::ValidationError);

  // Options given several files each.
  app.clear();
  EXPECT_THROW(parse({"-c", files[0], files[1], "-c", files[2], files[3]}), CLI::ValidationError);
  EXPECT_EQ(first, 0) << "a configuration file was read before the number of files was checked";

  // A single option given all the files takes as many as it can, and the remaining one is an unexpected argument.
  app.clear();
  EXPECT_THROW(parse({"-c", files[0], files[1], files[2], files[3]}), CLI::ParseError);
}
