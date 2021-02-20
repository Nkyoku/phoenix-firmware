#pragma once

#include <QtCore/QString>
#include <QtCore/QStringList>
#include <initializer_list>
#include <array>

static const QString& boolToString(bool value) {
    static const QString true_text = "true";
    static const QString false_text = "false";
    return value ? true_text : false_text;
}

template <typename... Args>
static const QString boolListToString(const QStringList& text_list, Args... args) {
    static const QString none_text = "none";
    static const QString separator_text = ", ";
    QStringList result;
    auto it = text_list.begin();
    for (bool value : std::initializer_list<bool>{args...}) {
        if (value) {
            result.append(*it);
        }
        ++it;
    }
    return result.empty() ? none_text : result.join(separator_text);
}

template <size_t N>
static const QString boolArrayToString(const QStringList& text_list, const std::array<bool, N> &values) {
    static const QString none_text = "none";
    static const QString separator_text = ", ";
    QStringList result;
    auto it = text_list.begin();
    for (bool value : values) {
        if (value) {
            result.append(*it);
        }
        ++it;
    }
    return result.empty() ? none_text : result.join(separator_text);
}
