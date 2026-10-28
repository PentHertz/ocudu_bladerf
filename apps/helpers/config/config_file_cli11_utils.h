// SPDX-FileCopyrightText: Copyright (C) 2026 OCUDU contributors
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI
// Portions of this file may implement 3GPP specifications, which may be subject to additional licensing requirements.

#pragma once

#include "CLI/CLI11.hpp"
#include "fmt/format.h"

namespace ocudu {
namespace app_helpers {

/// [Implementation defined] Maximum number of configuration files that can be passed in the command line.
constexpr unsigned MAX_NOF_CONFIG_FILES = 100;

/// \brief Adds the configuration file option, which reads the application configuration from one or more files.
///
/// The files are read in the order they are given, and a parameter set in a later file takes precedence over the same
/// parameter set in an earlier one. Parameters given in the command line take precedence over all the files.
///
/// When more files than \c max_nof_files are given, CLI11 keeps the last ones and silently drops the first ones.
/// The option therefore checks the number of files and makes the parsing fail, before any file is read, if it is
/// exceeded.
///
/// \param[in] app           CLI11 application.
/// \param[in] max_nof_files Maximum number of configuration files.
/// \param[in] option_name   Option name, in CLI11 format.
/// \param[in] description   Option description.
/// \return The added option.
inline CLI::Option* add_config_file_option(CLI::App&          app,
                                           unsigned           max_nof_files = MAX_NOF_CONFIG_FILES,
                                           const std::string& option_name   = "-c,",
                                           const std::string& description   = "Read config from file")
{
  CLI::Option* option = app.set_config(option_name, "", description, false)->expected(1, max_nof_files);

  // The validator runs on every given file name before the files are read, when the option still counts all of them.
  option->check(CLI::Validator(
      [option, max_nof_files](const std::string& /* file_name */) -> std::string {
        if (option->count() > max_nof_files) {
          return fmt::format(
              "at most {} configuration files are supported, {} were given", max_nof_files, option->count());
        }
        return {};
      },
      ""));

  return option;
}

} // namespace app_helpers
} // namespace ocudu
