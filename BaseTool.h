#ifndef BASE_TOOL_H
#define BASE_TOOL_H

#include <iostream>
#include <QObject>

#define DELETE_OBJECT(obj)                 \
    {                                      \
        if (obj != nullptr)                \
        {                                  \
            delete obj;                    \
            obj = nullptr;                 \
        }                                  \
    }

#define DELETE_QOBJECT(obj)                \
    {                                      \
        if (obj && obj->parent())          \
        {                                  \
            delete obj;                    \
            obj = nullptr;                 \
        }                                  \
    }

#endif