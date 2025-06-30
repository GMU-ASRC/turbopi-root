#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack
import argparse
import math
import random
import numpy as np

import hiwonder_common.statistics_tools as st
from hiwonder_common.camera_binary_program import range_bgr
import hiwonder_common.camera_binary_program as camera_binary_program
import hiwonder_common.jsontools as jst

import casPYan
import casPYan.ende.rate as ende

# typing
from typing import Any

DEFAULT_NETWORK_PATH = '/home/pi/networks/250423-031032-connorsim_snn_eons-v01.json'


def bool_to_one_hot(x: bool):
    return (0, 1) if x else (1, 0)


b2oh = bool_to_one_hot


class SNNMillingProgram(camera_binary_program.CameraBinaryProgram):
    def __init__(self,
        args,
        post_init=True, board=None, name=None, disable_logging=False,
        network=None,
    ) -> None:
        super().__init__(args, post_init=False, board=board, name=name, disable_logging=disable_logging)

        self.boolean_detection_averager = st.Average(2)

        self.network = None
        self.json_net: dict | None = None
        self.neuro_tpc: int = None  # type: ignore[reportAttributeAccessIssue]
        self.load_network(args.network)

        assert self.neuro_tpc is not None
        self.encoders = [ende.RateEncoder(self.neuro_tpc, [0.0, 1.0]) for _ in range(2)]
        self.decoders = [ende.RateDecoder(self.neuro_tpc, [0.0, 1.0]) for _ in range(4)]

        if post_init:
            self.startup_beep()

    def load_network(self, path_or_net):
        self.json_net = self.read_json(path_or_net)
        try:
            self.neuro_tpc = self.json_net['Associated_Data']['application']['encoder_ticks']
        except KeyError:
            self.neuro_tpc = 10
            print(f"WARNING: Could not read `encoder_ticks` from network. Using default value of {self.neuro_tpc}")
        self.set_network(self.convert_json_to_caspyan(self.json_net))

    @staticmethod
    def read_json(path_or_net) -> dict:
        return jst.smartload(path_or_net)

    @staticmethod
    def convert_json_to_caspyan(network: dict):
        return casPYan.network.network_from_json(network)

    def set_network(self, network):
        self.network = network
        self.nodes = list(network.nodes.values())

    def get_input_spikes(self, input_vector):
        input_slice = input_vector[:len(self.encoders)]
        return [enc.get_spikes(x) for enc, x in zip(self.encoders, input_slice)]
        # returns a vector of list of spikes for each node

    def apply_spikes(self, spikes_per_node):
        for node, spikes in zip(self.network.inputs, spikes_per_node):
            node.intake += spikes

    def decode_output(self):
        return [dec.decode(node.history) for dec, node in zip(self.decoders, self.network.outputs)]

    def run(self, ticks: int):
        casPYan.network.run(self.nodes, ticks)

    def startup_beep(self):
        self.buzzfor(.03, .05)
        self.buzzfor(.03, .05)

    def control(self):
        spikes_per_node = self.get_input_spikes(b2oh(self.smoothed_detected))
        self.apply_spikes(spikes_per_node)
        self.run(5)
        self.run(self.neuro_tpc)
        # v0, v1, w0, w1 = self.decode_output()
        data = self.decode_output()
        # three bins. One for +v, -v, omega.
        v = 0.2 * (data[1] - data[0])
        w = 2.0 * (data[3] - data[2])

        # convert w from rad to deg
        # w_rad = w
        # w = math.degrees(w_rad)
        fspd_power = math.copysign(max(0, st.fmap(abs(v), 0.124, 0.276, 35, 100)), v)
        turn_power = math.copysign(max(0, st.fmap(abs(w), 0.3, 2.6, 0.25, 2)), w)
        # print(data, v, w, fspd_power, turn_power)

        # print(v, w)
        self.set_rgb('green' if bool(self.detected) else 'red')
        if not self.dry_run:
            self.move(fspd_power, 90, turn_power)


def get_parser(parser: argparse.ArgumentParser, subparsers=None):
    parser.add_argument('--network', default=DEFAULT_NETWORK_PATH)
    return parser, subparsers


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser, subparsers = camera_binary_program.get_parser(parser)
    parser, subparsers = get_parser(parser)
    args = parser.parse_args()

    program = SNNMillingProgram(args,)
    program.main()
