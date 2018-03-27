QT += core
QT -= gui

CONFIG += c++11

TARGET = Registration
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    cfilter.cpp \
    regmesh.cpp \
    cinout.cpp

HEADERS += \
    cfilter.h \
    regmesh.h \
    cinout.h
