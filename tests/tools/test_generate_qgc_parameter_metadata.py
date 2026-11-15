#!/usr/bin/env python3
"""Regression tests for the Flight-to-QGroundControl metadata generator."""

import unittest

from tools.generate_qgc_parameter_metadata import (
    build_model_catalog_document,
    build_parameter_document,
    validate_profile_alignment,
    _estimator_profile,
    _estimator_status,
)


class QgcParameterMetadataGeneratorTests(unittest.TestCase):
    def setUp(self):
        self.catalog = {
            "sha256": "test-catalog-sha256",
            "profile": "QUAD_X4",
            "parameters": [
                {
                    "name": "VEH_MASS",
                    "group": "机体模型",
                    "title": "整机质量",
                    "description": "Model mass.",
                    "units": "kg",
                    "minimum": 0.1,
                    "maximum": 50.0,
                    "value": 1.5,
                    "writable": True,
                    "file": "vehicle_model.yaml",
                    "path": ["mass_kg"],
                    "kind": "scalar",
                },
                {
                    "name": "CTL_RATE_KP_X",
                    "group": "姿态角速度环",
                    "title": "Rate P X",
                    "description": "Rate proportional gain.",
                    "units": "Nm/rad/s",
                    "minimum": 0.0,
                    "maximum": 20.0,
                    "value": 0.12,
                    "writable": True,
                    "file": "controller_tuning.yaml",
                    "path": ["inner_loop", "rate_kp_nm_per_radps", 0],
                    "kind": "vector",
                },
            ],
        }
        self.model = {
            "profile": "QUAD_X4",
            "status": "SIM_ONLY",
            "source": "Synthetic fixture.",
            "hardware_thrust_curve": "TBD: no hardware calibration",
            "vehicle_family": "MULTIROTOR",
            "model_variants": ["QUAD_X4"],
            "control_law": "MPC_OUTER_SO3_ATTITUDE_RATE_PID",
        }
        self.controller = {"profile": "QUAD_X4", "status": "SIM_ONLY"}
        self.parameter_document = build_parameter_document(self.catalog, self.model)

    def test_parameter_domains_and_model_scope_are_generated(self):
        parameters = self.parameter_document["parameters"]
        self.assertEqual(parameters[0]["domain"], "VEHICLE_MODEL")
        self.assertEqual(parameters[1]["domain"], "CONTROL")
        self.assertEqual(parameters[0]["modelProfiles"], ["QUAD_X4"])
        self.assertEqual(self.parameter_document["modelProfiles"], ["QUAD_X4"])
        self.assertEqual(self.parameter_document["version"], 2)
        self.assertEqual(parameters[0]["vehicleFamily"], "MULTIROTOR")
        self.assertEqual(parameters[0]["modelVariants"], ["QUAD_X4"])
        self.assertEqual(parameters[0]["controlLaw"], "MPC_OUTER_SO3_ATTITUDE_RATE_PID")
        self.assertEqual(parameters[0]["tuningPolicy"], "GROUND_ADJUSTABLE")
        self.assertEqual(parameters[0]["updatePolicy"], "FIRMWARE_CONFIRMATION")
        self.assertEqual(parameters[0]["dependencies"], [])
        self.assertEqual(parameters[0]["validationRules"], [])
        self.assertEqual(parameters[0]["sourceFile"], "vehicle_model.yaml")
        self.assertEqual(parameters[0]["sourcePath"], "mass_kg")
        self.assertEqual(parameters[0]["sourceKind"], "scalar")
        self.assertEqual(parameters[1]["sourcePath"], "inner_loop.rate_kp_nm_per_radps[0]")

    def test_reboot_requirement_is_unknown_unless_source_declares_it(self):
        self.assertNotIn("rebootRequired", self.parameter_document["parameters"][0])
        self.assertNotIn("rebootRequired", self.parameter_document["parameters"][1])
        self.assertNotIn("rebootRequired", self.parameter_document["parameters"][-1])

        catalog = dict(self.catalog)
        catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
        catalog["parameters"][0]["reboot_required"] = False
        catalog["parameters"][1]["rebootRequired"] = True
        document = build_parameter_document(catalog, self.model)
        self.assertIs(document["parameters"][0]["rebootRequired"], False)
        self.assertIs(document["parameters"][1]["rebootRequired"], True)
        self.assertEqual(document["parameters"][1]["updatePolicy"], "REBOOT_REQUIRED")

    def test_model_descriptor_is_required_and_profile_scoped(self):
        model = dict(self.model)
        model.pop("control_law")
        with self.assertRaises(ValueError):
            build_parameter_document(self.catalog, model)

        model = dict(self.model, model_variants=["HEX_X6"])
        with self.assertRaises(ValueError):
            build_parameter_document(self.catalog, model)

    def test_read_only_source_parameter_gets_read_only_policy(self):
        catalog = dict(self.catalog)
        catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
        catalog["parameters"][1]["writable"] = False
        document = build_parameter_document(catalog, self.model)
        parameter = document["parameters"][1]
        self.assertEqual(parameter["tuningPolicy"], "READ_ONLY")
        self.assertTrue(parameter["readOnly"])

    def test_source_path_must_be_explicit_and_well_formed(self):
        catalog = dict(self.catalog)
        catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
        catalog["parameters"][0]["path"] = ["outer_loop", ""]
        with self.assertRaises(ValueError):
            build_parameter_document(catalog, self.model)

    def test_unknown_parameter_dependencies_are_rejected(self):
        catalog = dict(self.catalog)
        catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
        catalog["parameters"][0]["dependencies"] = ["MISSING_PARAMETER"]
        with self.assertRaises(ValueError):
            build_parameter_document(catalog, self.model)

    def test_validation_rules_must_have_a_type(self):
        catalog = dict(self.catalog)
        catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
        catalog["parameters"][0]["validationRules"] = [{}]
        with self.assertRaises(ValueError):
            build_parameter_document(catalog, self.model)

    def test_supported_validation_rules_are_preserved(self):
        catalog = dict(self.catalog)
        catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
        catalog["parameters"][0]["dependencies"] = ["CTL_RATE_KP_X"]
        catalog["parameters"][0]["validationRules"] = [
            {"type": "GREATER_THAN", "value": 0.5},
            {"type": "REQUIRES_PARAMETER_VALUE", "parameter": "CTL_RATE_KP_X", "value": 1.0},
        ]
        document = build_parameter_document(catalog, self.model)
        parameter = document["parameters"][0]
        self.assertEqual(parameter["dependencies"], ["CTL_RATE_KP_X"])
        self.assertEqual(parameter["validationRules"][0]["type"], "GREATER_THAN")
        self.assertEqual(parameter["validationRules"][1]["parameter"], "CTL_RATE_KP_X")

    def test_unsupported_or_malformed_validation_rules_are_rejected(self):
        invalid_rules = [
            [{"type": "EXECUTE_SCRIPT", "value": 1.0}],
            [{"type": "GREATER_THAN", "value": True}],
            [{"type": "MULTIPLE_OF", "value": 0.0}],
            [{"type": "GREATER_THAN", "value": 1.0, "unexpected": True}],
            [{"type": "REQUIRES_PARAMETER_VALUE", "parameter": "CTL_RATE_KP_X", "value": 1.0}],
        ]
        for rules in invalid_rules:
            catalog = dict(self.catalog)
            catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
            catalog["parameters"][0]["validationRules"] = rules
            with self.subTest(rules=rules), self.assertRaises(ValueError):
                build_parameter_document(catalog, self.model)

    def test_invalid_parameter_policy_metadata_is_rejected(self):
        catalog = dict(self.catalog)
        catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
        catalog["parameters"][0]["dependencies"] = ["", "CTL_RATE_KP_X"]
        with self.assertRaises(ValueError):
            build_parameter_document(catalog, self.model)

    def test_invalid_reboot_requirement_is_rejected(self):
        catalog = dict(self.catalog)
        catalog["parameters"] = [dict(item) for item in self.catalog["parameters"]]
        catalog["parameters"][0]["reboot_required"] = "false"
        with self.assertRaises(ValueError):
            build_parameter_document(catalog, self.model)

    def test_model_sources_must_share_the_same_profile(self):
        with self.assertRaises(ValueError):
            validate_profile_alignment(
                self.catalog,
                self.model,
                {"profile": "OTHER_MODEL"},
                "QUAD_X4",
            )

    def test_sim_only_profile_is_not_hardware_eligible(self):
        catalog = build_model_catalog_document(
            self.catalog,
            self.model,
            self.controller,
            "DRAFT_PENDING_DATASET",
            self.parameter_document,
        )
        profile = catalog["profiles"][0]
        self.assertEqual(profile["status"], "SIM_ONLY")
        self.assertFalse(profile["hardwareEligible"])
        self.assertEqual(profile["parameterWriteScope"], "SIL_ONLY")
        self.assertEqual(catalog["schema_version"], 2)
        self.assertEqual(profile["vehicleFamily"], "MULTIROTOR")
        self.assertEqual(profile["modelVariants"], ["QUAD_X4"])
        self.assertEqual(profile["controlLaw"], "MPC_OUTER_SO3_ATTITUDE_RATE_PID")
        self.assertIn("SIMULATION", profile["parameterDomains"])

    def test_hardware_eligibility_requires_all_three_validated_sources(self):
        model = dict(self.model, status="HARDWARE_VALIDATED", hardware_thrust_curve="calibrated")
        controller = {"status": "HARDWARE_VALIDATED"}
        catalog = build_model_catalog_document(
            self.catalog,
            model,
            controller,
            "HARDWARE_VALIDATED",
            self.parameter_document,
        )
        self.assertTrue(catalog["profiles"][0]["hardwareEligible"])
        self.assertEqual(catalog["profiles"][0]["parameterWriteScope"], "HARDWARE_VALIDATED")

    def test_estimator_yaml_profile_and_status_are_parsed(self):
        text = "schema_version: 1\nprofile: QUAD_X4\nstatus: DRAFT_PENDING_DATASET\n"
        self.assertEqual(_estimator_profile(text), "QUAD_X4")
        self.assertEqual(_estimator_status(text), "DRAFT_PENDING_DATASET")


if __name__ == "__main__":
    unittest.main(verbosity=2)
