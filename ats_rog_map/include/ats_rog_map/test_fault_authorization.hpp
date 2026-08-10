// Copyright 2026

#ifndef ATS_ROG_MAP__TEST_FAULT_AUTHORIZATION_HPP_
#define ATS_ROG_MAP__TEST_FAULT_AUTHORIZATION_HPP_

#include <string>
#include <vector>

namespace ats_rog_map
{

/// @brief Outcome of screening a runtime parameter request against the
///        `enable_test_fault_injection` startup authorization gate.
struct TestFaultAuthorizationVerdict
{
  bool accepted{true};
  std::string reason;
};

/// @brief Reject any test-only fault parameter while the startup gate is false,
///        and reject every attempt to move the gate itself at runtime.
/// @param gate_enabled  Value latched from `enable_test_fault_injection` at
///                      construction time.  It is deliberately not re-read per
///                      request so the gate cannot be raised by a live client.
/// @param requested_names  Names carried by one `SetParameters` request.
/// @param guarded_names  Test-only parameter names owned by this node.
/// @return `accepted == false` with a named reason when the request must be
///         refused outright.  A refusal means nothing is applied: the map is
///         never silently cleared and no fixture becomes active.
inline TestFaultAuthorizationVerdict screenTestFaultParameters(
  const bool gate_enabled,
  const std::vector<std::string> & requested_names,
  const std::vector<std::string> & guarded_names)
{
  TestFaultAuthorizationVerdict verdict;
  for (const auto & name : requested_names) {
    if (name == "enable_test_fault_injection") {
      verdict.accepted = false;
      verdict.reason =
        "enable_test_fault_injection is a startup-only authorization gate and cannot be "
        "changed at runtime";
      return verdict;
    }
    if (gate_enabled) {
      continue;
    }
    for (const auto & guarded : guarded_names) {
      if (name == guarded) {
        verdict.accepted = false;
        verdict.reason = "test fault parameter '" + name +
          "' refused: enable_test_fault_injection=false at startup, so no test fixture may "
          "alter map semantics";
        return verdict;
      }
    }
  }
  return verdict;
}

}  // namespace ats_rog_map

#endif  // ATS_ROG_MAP__TEST_FAULT_AUTHORIZATION_HPP_
