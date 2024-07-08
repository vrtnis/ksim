# mypy: disable-error-code="import-not-found"
"""Updates the Stompy model."""

import argparse
from dataclasses import dataclass

from ksim.scripts.run_onshape_to_urdf import run_onshape_to_urdf


@dataclass
class Artifact:
    url: str
    override_central_node: str | None = None


ARTIFACTS: dict[str, Artifact] = {
    "stompy_arm_7dof": Artifact(
        url=(
            "https://cad.onshape.com/documents/afaee604f6ca311526a6aec8/"
            "w/29af84cb974c2d825b71de39/e/949e5b6cd071fee000e2c296"
        ),
        override_central_node="upper_left_arm_1_arm_part_1_1",
    ),
    "stompy_arm_5dof": Artifact(
        url=(
            "https://cad.onshape.com/documents/afaee604f6ca311526a6aec8/"
            "w/29af84cb974c2d825b71de39/e/4fef6bce7179a665e62b03ba"
        ),
        override_central_node="upper_left_arm_1_arm_part_1_1",
    ),
    "stompy_7dof": Artifact(
        url=(
            "https://cad.onshape.com/documents/71f793a23ab7562fb9dec82d/"
            "w/e879bcf272425973f9b3d8ad/e/1a95e260677a2d2d5a3b1eb3"
        ),
    ),
    "stompy_5dof": Artifact(
        url=(
            "https://cad.onshape.com/documents/71f793a23ab7562fb9dec82d/"
            "w/e879bcf272425973f9b3d8ad/e/e07509571989528061c06a08"
        ),
    ),
    "stompy_torso_7dof": Artifact(
        url=(
            "https://cad.onshape.com/documents/77f178833017912bdc0eaaa8/"
            "w/958f1684049f1413f6dd7831/e/348cad65143f892a40e33206"
        ),
        override_central_node="torso_1_hip_mount_1",
    ),
    "stompy_torso_5dof": Artifact(
        url=(
            "https://cad.onshape.com/documents/77f178833017912bdc0eaaa8/"
            "w/958f1684049f1413f6dd7831/e/363614f1fc7c150003de244c"
        ),
        override_central_node="torso_1_hip_mount_1",
    ),
}


def main() -> None:
    parser = argparse.ArgumentParser(description="Update the Stompy model.")
    parser.add_argument("key", choices=ARTIFACTS.keys())
    args = parser.parse_args()

    artifact = ARTIFACTS[args.key]

    run_onshape_to_urdf(
        model_url=artifact.url,
        output_dir=args.key,
        override_central_node=artifact.override_central_node,
    )


if __name__ == "__main__":
    # python -m ksim.scripts.update_stompy_s3
    main()
