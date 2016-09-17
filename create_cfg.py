#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file       environ.py
@author     Allen Woods
@date       2016-07-29
@version    16-7-29 下午2:51 ???
SUMO simulation environment
"""
import os
import subprocess
import numpy as np
from subprocess import PIPE
import sys
from itertools import cycle

from lxml import etree

sumo_root = 'sumo-0.27.1'

try:
    print('SUMO_ROOT: %s' % sumo_root)
    sumo_home = os.path.join(sumo_root, 'tools')
    sys.path.append(sumo_home)
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "Please declare environment variable 'SUMO_HOME' as the root directory "
        "of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")


class SumoCfg:
    def __init__(self, data_dir, net_name, xnumber, ynumber,
                 xlength=1000, ylength=1000, net_type='grid', tls_type='static', rouprob=10,
                 period=None, binominal=None, steps=3600):
        """
        Initialize SUMO environment.
        :param data_dir: where XML saved
        :param net_name: name of network
        :param xnumber: nodes' number on x axis
        :param ynumber: nodes' number on y axis
        :param xlength: length of each edge on x axis
        :param ylength: length of each edge on y axis
        :param net_type: type of network, sumo support 'grid', 'spider', 'random'.
        :param tls_type: style of traffic light(static, actuated)
        :param rouprob: probability to generate a route begin/end at edge without precursor/successor
        :param period: generates vehicles with a constant period and arrival rate of (1/period) per second.
        :param binominal: the arrivals will be randomized using a binomial distribution where n (the maximum number of simultaneous arrivals) is given
        :param steps: steps generate by randomtrips
        """

        self.netname = net_name
        self.data_dir = data_dir
        self.net_dir = os.path.join(self.data_dir, self.netname)
        if not os.path.isdir(self.net_dir):
            os.makedirs(self.net_dir)
        self.output_dir = os.path.join(self.net_dir, 'output', )
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        self.xnumber = str(xnumber)
        self.ynumber = str(ynumber)
        self.xlength = str(xlength)
        self.ylength = str(ylength)
        self.rouprob = str(rouprob)
        self.period = period
        self.binominal = binominal
        self.tlstype = tls_type
        self.nettype = net_type
        self.steps = str(steps)
        self.netfile = os.path.join(self.net_dir, self.netname + '.net.xml')
        self.tripfile = os.path.join(self.net_dir, self.netname + '.trip.xml')
        self.roufile = os.path.join(self.net_dir, self.netname + '.rou.xml')
        self.detectors = [os.path.join(self.net_dir, self.netname + '_e%d.add.xml' % (i + 1))
                          for i in range(3)]
        self.sumocfg = os.path.join(self.net_dir, self.netname + '.sumo.cfg')
        self.sumocfg_nodet = os.path.join(self.net_dir, self.netname + '_nodet.sumo.cfg')
        self.summary_file = os.path.join(self.output_dir, self.netname + '_summary.xml')

    def iscreated(self):
        """
        Check if the XMLs have been created
        :return:
        """
        return os.path.isfile(self.roufile) and os.path.isfile(self.netfile)

    def init(self, withdet=False):
        """
        Create the XML needed for simulation
        :param init_time: strategy to controll traffic light
        :param gui: with GUI
        :param withdet: with detector(e1,e2,e3)
        :return:
        """
        self.gen_network(self.xnumber, self.ynumber, self.xlength, self.ylength,
                         nettype=self.nettype, tlstype=self.tlstype)  # Set length to 400
        self.gen_randomtrips(self.rouprob, endtime=self.steps, period=self.period,
                             binomial=self.binominal)  # Set edge prop to 10

        # if gui:
        #     sumoBinary = checkBinary('sumo-gui')
        # else:
        #     sumoBinary = checkBinary('sumo')
        if withdet:
            self.sumocfg = self.gen_sumocfg(withdetector=True)
            self.gen_detectors()
            # sumocfg = self.sumocfg
        else:
            self.sumocfg_nodet = self.gen_sumocfg(withdetector=False)
            # sumocfg = self.sumocfg_nodet
            # sumo_env = os.environ.copy()
            # sumo_env['SUMO_HOME'] = sumo_root
            # sumoProcess = subprocess.Popen([sumoBinary, '-c', sumocfg, '--remote-port', str(port),
            #                                 '--summary', self.summary_file],
            #                                 env=sumo_env, stdout=PIPE, stderr=PIPE)
            # sumoCMD = [sumoBinary, '-c', sumocfg, '--summary', self.summary_file,
            #            '--time-to-teleport', '-1']
            # sumoProcess.wait()
            # return sumoCMD, sumo_env

    def gen_network(self, xnumber, ynumber, xlength, ylength,
                    nettype='grid', tlstype='static'):
        """
        Generate network model
        :param xnumber: nodes on x axis
        :param ynumber: nodes on y axis
        :param xlength: length of each edge on x axis
        :param ylength: length of each edge on y axis
        :param nettype: type of network, sumo support 'grid', 'spider', 'random'.
        :param tlstype:
        :return:
        """
        if int(xnumber) < 2 or int(ynumber) < 2:
            if xnumber == ynumber == str(1) and xlength == ylength:
                self.gen_intersection(xlength, tlstype)
                return 0
            else:
                raise ValueError('Grid sharp is not supported(yet)')
        netgenerate = checkBinary('netgenerate')
        netgenProcessor = subprocess.Popen([netgenerate, '--%s' % nettype,
                                            '--grid.x-number', xnumber, '--grid.y-number', ynumber,
                                            '--grid.x-length', xlength, '--grid.y-length', ylength,
                                            '--tls.guess', 'true', '--tls.default-type', tlstype,
                                            '--default.lanenumber', '2',
                                            '--check-lane-foes.all', 'true',
                                            '--%s.attach-length' % nettype, xlength,
                                            '--plain-output-prefix',
                                            os.path.join(self.data_dir, self.netname, self.netname),
                                            '-o', self.netfile], stdout=sys.stdout, stderr=sys.stderr)

    def gen_randomtrips(self, rouprob, endtime, period=None, binomial=None,
                        trip_attrib="departLane=\"best\" departSpeed=\"max\" departPos=\"random\""):
        """
        Generate random trip on given network
        Warning: Sometimes routefile might not be created
        :param rouprob: probability to generate a route begin/end at edge without precursor/successor
        :param endtime: end time of the simulation
        :param period: random seed
        :param trip_attrib: additional attribute of random trips
        :return:
        """
        if period is None:
            period = str(np.random.uniform(0.2, 1.2))
        if binomial is None:
            binomial = str(np.random.randint(1, 10))
        rantrip_generator = os.path.join(sumo_home, 'randomTrips.py')
        gentripProcessor = subprocess.Popen(['python', rantrip_generator, '-n', self.netfile,
                                             '-e', endtime, '--binomial', binomial, '-p', period,
                                             '--fringe-factor', rouprob,
                                             '--trip-attributes', trip_attrib, '-o', self.tripfile,
                                             '-r', self.roufile],
                                            stdout=sys.stdout, stderr=sys.stderr)

    def gen_detectors(self):
        """
        Generate Detector on the given network
        :return:
        """
        e1generator = os.path.join(sumo_home, 'output', 'generateTLSE1Detectors.py')
        e2generator = os.path.join(sumo_home, 'output', 'generateTLSE2Detectors.py')
        e3generator = os.path.join(sumo_home, 'output', 'generateTLSE3Detectors.py')
        det_outputs = ['output/' +
                       'e%d_output.xml' % (i + 1) for i in range(3)]
        e_generators = [e1generator, e2generator, e3generator]
        paras = zip(e_generators, cycle(['-n']), cycle([self.netfile]),
                    cycle(['-o']), self.detectors, cycle(['-r']), det_outputs)
        for p in paras:
            p = list(p)
            d = subprocess.Popen(p, stdout=sys.stdout, stderr=sys.stderr)

    def gen_sumocfg(self, withdetector=False):
        """
        Generate sumo.cfg file for sumo to load
        :param withdetector: controll whether the detector should be add in the file or not
        :return:
        """
        xsd_file = "http://sumo.dlr.de/xsd/sumoConfiguration.xsd"
        conf_root = etree.Element("configuration",
                                  nsmap={'xsi': "http://www.w3.org/2001/XMLSchema-instance"},
                                  attrib={'noNamespaceSchemaLocation': xsd_file})
        # Set Input file
        conf_input = etree.SubElement(conf_root, 'input')
        netfile = self.netname + '.net.xml'  # Use relative address
        roufile = self.netname + '.rou.xml'  # makes configuaration portable
        detectors = [self.netname + '_e%d.add.xml' % (i + 1) for i in range(3)]
        input_netfile = etree.SubElement(conf_input, 'net-file')
        input_netfile.set('value', netfile)
        input_roufile = etree.SubElement(conf_input, 'route-files')
        input_roufile.set('value', roufile)
        if withdetector:
            input_addfile = etree.SubElement(conf_input, 'additional-files')
            input_addfile.set('value', " ".join(detectors))
        # Set Output file
        conf_output = etree.SubElement(conf_root, 'output')
        netstatfile = 'output/' + self.netname + '_netstate.sumo.tr'
        tripinfo = 'output/' + '/' + self.netname + '_tripinfo.xml'
        vehroute = 'output/' + self.netname + '_vehroutes.xml'
        queue = 'output/' + self.netname + '_queues.xml'
        output_nets = etree.SubElement(conf_output, 'netstate-dump')
        output_nets.set('value', netstatfile)
        output_tripinfo = etree.SubElement(conf_output, 'tripinfo-output')
        output_tripinfo.set('value', tripinfo)
        output_vehroute = etree.SubElement(conf_output, 'vehroute-output')
        output_vehroute.set('value', vehroute)
        output_queue = etree.SubElement(conf_output, 'queue-output')
        output_queue.set('value', queue)
        # Set Time
        conf_time = etree.SubElement(conf_root, 'time')
        time_begin = etree.SubElement(conf_time, 'begin')
        time_begin.set('value', '0')
        time_end = etree.SubElement(conf_time, 'end')
        time_end.set('value', '3600')
        time_roustep = etree.SubElement(conf_time, 'route-steps')
        time_roustep.set('value', '-1')
        # Set Time to teleport
        conf_teleport = etree.SubElement(conf_root, 'time-to-teleport')
        conf_teleport.set('value', '-1')

        # Write to sumo.cfg
        conf_tree = etree.ElementTree(conf_root)
        if withdetector:
            sumocfg_file = self.sumocfg
        else:
            sumocfg_file = self.sumocfg_nodet
        conf_tree.write(sumocfg_file, pretty_print=True, xml_declaration=True, encoding='utf-8')
        return sumocfg_file

    def gen_intersection(self, edgelen, tlstype='static'):
        length = int(edgelen)
        cross_nodes_file = os.path.join(self.net_dir, '%s.nod.xml' % self.netname)
        cross_edges_file = os.path.join(self.net_dir, '%s.edg.xml' % self.netname)
        # Generate nodes
        node_xsd_file = "http://sumo.dlr.de/xsd/nodes_file.xsd"
        nodes_root = etree.Element("nodes",
                                   nsmap={'xsi': "http://www.w3.org/2001/XMLSchema-instance"},
                                   attrib={'noNamespaceSchemaLocation': node_xsd_file})
        cross_node = etree.SubElement(nodes_root, 'node')
        cross_node.set('id', 'cross')
        cross_node.set('x', '0')
        cross_node.set('y', '0')
        cross_node.set('type', 'traffic_light')
        cross_node.set('tlType', tlstype)
        n_node = etree.SubElement(nodes_root, 'node')
        n_node.set('id', 'north')
        n_node.set('x', '0')
        n_node.set('y', str(length))
        n_node.set('type', 'priority')
        s_node = etree.SubElement(nodes_root, 'node')
        s_node.set('id', 'south')
        s_node.set('x', '0')
        s_node.set('y', str(-length))
        s_node.set('type', 'priority')
        e_node = etree.SubElement(nodes_root, 'node')
        e_node.set('id', 'east')
        e_node.set('x', str(length))
        e_node.set('y', '0')
        e_node.set('type', 'priority')
        w_node = etree.SubElement(nodes_root, 'node')
        w_node.set('id', 'west')
        w_node.set('x', str(-length))
        w_node.set('y', '0')
        w_node.set('type', 'priority')
        nodes_tree = etree.ElementTree(nodes_root)
        # print(cross_nodes_file)
        nodes_tree.write(cross_nodes_file,
                         pretty_print=True, xml_declaration=True, encoding='utf-8')
        # Create edges
        edges_xsd_file = "http://sumo.dlr.de/xsd/edges_file.xsd"
        edges_root = etree.Element("edges",
                                   nsmap={'xsi': "http://www.w3.org/2001/XMLSchema-instance"},
                                   attrib={'noNamespaceSchemaLocation': edges_xsd_file})
        create_edges(edges_root, 'north', 'cross')
        create_edges(edges_root, 'south', 'cross')
        create_edges(edges_root, 'east', 'cross')
        create_edges(edges_root, 'west', 'cross')
        edges_tree = etree.ElementTree(edges_root)
        edges_tree.write(cross_edges_file,
                         pretty_print=True, xml_declaration=True, encoding='utf-8')
        netconvert = checkBinary('netconvert')
        netconvertor = subprocess.Popen([netconvert,
                                         '--node-files', cross_nodes_file,
                                         '--edge-files', cross_edges_file,
                                         '--output-file', self.netfile],
                                        stdout=sys.stdout, stderr=sys.stderr)


def create_edges(root, node1_id, node2_id):
    edge1 = etree.SubElement(root, 'edge')
    edge1.set('id', '%s_to_%s' % (node1_id, node2_id))
    edge1.set('from', node1_id)
    edge1.set('to', node2_id)
    edge1.set('numLanes', '2')
    edge1.set('speed', '13.9')  # Default Speed 13.9m/s
    edge2 = etree.SubElement(root, 'edge')
    edge2.set('id', '%s_to_%s' % (node2_id, node1_id))
    edge2.set('from', node2_id)
    edge2.set('to', node1_id)
    edge2.set('numLanes', '2')
    edge2.set('speed', '13.9')  # Default Speed 13.9m/s


if __name__ == '__main__':
    # Test funtion and prepare the simulation environments.
    data_dir = os.path.join(os.getcwd(), 'tmp')
    for i in range(100000):
        sumo = SumoCfg(data_dir, 'train_sim%.6d' % i, 1, 1)
        sumo.init()
        # sumo.gen_sumocfg()
        # cmd, env = sumo.get_start_cmd('static')
        # print(cmd)
