# -*- coding: utf-8 -*-

# Copyright (C) 2016, Maximilian Köhl <mail@koehlma.de>

from lps.debugger import Debugger, INFO, WARNING, ERROR, SUCCESS


def log(message, kind=INFO):
    if Debugger.current:
        Debugger.current.print_message(message, kind)
    else:
        print(message)
