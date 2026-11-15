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
                },
            ],
        }
        self.model = {
            "profile": "QUAD_X4",
            "status": "SIM_ONLY",
            "source": "Synthetic fixture.",
            "hardware_thrust_curve": "TBD: no hardware calibration",
        }
        self.controller = {"profile": "QUAD_X4", "status": "SIM_ONLY"}
        self.parameter_document = build_parameter_document(self.catalog, self.model)

    def test_parameter_domains_and_model_scope_are_generated(self):
        parameters = self.parameter_document["parameters"]
        self.assertEqual(parameters[0]["domain"], "VEHICLE_MODEL")
        self.assertEqual(parameters[1]["domain"], "CONTROL")
        self.assertEqual(parameters[0]["modelProfiles"], ["QUAD_X4"])
        self.assertEqual(self.parameter_document["modelProfiles"], ["QUAD_X4"])

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
