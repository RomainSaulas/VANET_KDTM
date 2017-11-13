# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

# def options(opt):
#     pass

# def configure(conf):
#     conf.check_nonfatal(header_name='stdint.h', define_name='HAVE_STDINT_H')

def build(bld):
    module = bld.create_ns3_module('kdtm', ['core'])
    module.source = [
#        'model/kdtm.cc',
        'model/kdtm-ptable.cc',
        'model/kdtm-packet.cc',
        'model/kdtm-wqueue.cc',
#        'helper/kdtm-helper.cc'
        ]

    module_test = bld.create_ns3_module_test_library('kdtm')
    module_test.source = [
        'test/kdtm-test-suite.cc',
        ]

    headers = bld(features='ns3header')
    headers.module = 'kdtm'
    headers.source = [
#        'model/kdtm.h',
        'model/kdtm-ptable.h',
        'model/kdtm-packet.h',
        'model/kdtm-wqueue.h',
#        'helper/kdtm-helper.h',
        ]

    #if bld.env.ENABLE_EXAMPLES:
    #    bld.recurse('examples')

    # bld.ns3_python_bindings()

