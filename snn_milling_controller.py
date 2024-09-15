#!/usr/bin/python3
# coding=utf8
# from contextlib import ExitStack
import argparse
import math
import milling_controller
from milling_controller import BinaryProgram, range_bgr
import hiwonder_common.statistics_tools as st
import hiwonder_common.jsontools as jst

import casPYan
import casPYan.ende.rate as ende

# typing
from typing import Any

import warnings
try:
    import buttonman as buttonman
    buttonman.TaskManager.register_stoppable()
except ImportError:
    buttonman = None
    warnings.warn("buttonman was not imported, so no processes can be registered. This means the process can't be stopped by buttonman.",  # noqa: E501
                  ImportWarning, stacklevel=2)

DEFAULT_NETWORK_PATH = '/home/pi/networks/240914-174037-best.json'


def bool_to_one_hot(x: bool):
    return (0, 1) if x else (1, 0)


b2oh = bool_to_one_hot


class SNNMillingProgram(BinaryProgram):
    def __init__(self,
        args,
        dry_run: bool = False,
        board=None,
        lab_cfg_path=milling_controller.THRESHOLD_CFG_PATH,
        servo_cfg_path=milling_controller.SERVO_CFG_PATH,
        network=None,
        pause=False,
        startup_beep=True,
        exit_on_stop=True
    ) -> None:
        super().__init__(args, dry_run, board, lab_cfg_path, servo_cfg_path, pause, False, exit_on_stop)

        self.boolean_detection_averager = st.Average(2)

        self.network = None
        self.json_net: dict | None = None
        self.neuro_tpc: int | None = None
        self.load_network(network)

        assert self.neuro_tpc is not None
        self.encoders = [ende.RateEncoder(self.neuro_tpc, [0.0, 1.0]) for _ in range(2)]
        self.decoders = [ende.RateDecoder(self.neuro_tpc, [0.0, 1.0]) for _ in range(4)]

        if startup_beep:
            self.startup_beep()

    def load_network(self, path_or_net):
        try:
            self.json_net = self.read_json(path_or_net)
        except BaseException as err:
            self.json_net = None
        else:
            try:
                self.neuro_tpc = self.json_net['Associated_Data']['application']['proc_ticks']
            except KeyError:
                self.neuro_tpc = 10
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
        data = [int(round(x)) for x in data]
        # three bins. One for +v, -v, omega.
        v_mapping = [0.0, 100]
        w_mapping = [0.0, 0.5]
        v = v_mapping[data[1]] - v_mapping[data[0]]
        w = w_mapping[data[3]] - w_mapping[data[2]]

        fspd_power = v
        turn_power = w
        # print(data, v, w, fspd_power, turn_power)

        # print(v, w)
        self.set_rgb('green' if bool(self.detected) else 'red')
        if not self.dry_run:
            self.chassis.set_velocity(fspd_power, 90, turn_power)


def get_parser(parser: argparse.ArgumentParser, subparsers=None):
    parser.add_argument('--network', default=DEFAULT_NETWORK_PATH)
    return parser, subparsers


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser, subparsers = milling_controller.get_parser(parser)
    parser, subparsers = get_parser(parser)
    args = parser.parse_args()

    program = SNNMillingProgram(
        args,
        dry_run=args.dry_run,
        pause=args.startpaused,
        network=args.network,
    )
    program.main()
