#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace Test
{
    struct Case
    {
        const char *name = "";
        std::function<void()> run;
    };

    inline std::vector<Case> &registry()
    {
        static std::vector<Case> cases;

        return cases;
    }

    struct Registrar
    {
        Registrar(
            const char *name,
            std::function<void()> run)
        {
            registry().push_back(
                Case{name, run});
        }
    };

    inline void fail(
        const char *expression,
        const char *file,
        int line,
        const std::string &message = "")
    {
        std::ostringstream stream;

        stream << file << ":" << line
               << " assertion failed: "
               << expression;

        if (!message.empty())
        {
            stream << " (" << message << ")";
        }

        throw std::runtime_error(
            stream.str());
    }

    template <typename T, typename U>
    inline void expectEqual(
        const T &actual,
        const U &expected,
        const char *actualExpression,
        const char *expectedExpression,
        const char *file,
        int line)
    {
        if (!(actual == expected))
        {
            std::ostringstream message;

            message << actualExpression
                    << " != "
                    << expectedExpression;

            fail(
                actualExpression,
                file,
                line,
                message.str());
        }
    }

    inline void expectNear(
        float actual,
        float expected,
        float tolerance,
        const char *actualExpression,
        const char *expectedExpression,
        const char *file,
        int line)
    {
        if (std::fabs(actual - expected) > tolerance)
        {
            std::ostringstream message;

            message << actualExpression << "=" << actual
                    << ", " << expectedExpression << "="
                    << expected
                    << ", tolerance=" << tolerance;

            fail(
                actualExpression,
                file,
                line,
                message.str());
        }
    }

    inline int runAll()
    {
        int failed = 0;

        for (const Case &testCase : registry())
        {
            try
            {
                testCase.run();

                std::cout << "[PASS] "
                          << testCase.name
                          << '\n';
            }
            catch (const std::exception &error)
            {
                failed++;

                std::cerr << "[FAIL] "
                          << testCase.name
                          << ": "
                          << error.what()
                          << '\n';
            }
        }

        std::cout << registry().size() - failed
                  << "/"
                  << registry().size()
                  << " tests passed"
                  << '\n';

        return failed == 0 ? 0 : 1;
    }
}

#define TEST_CASE(name)                                                        \
    static void name();                                                        \
    static Test::Registrar registrar_##name(#name, name);                      \
    static void name()

#define EXPECT_TRUE(expression)                                                \
    do                                                                         \
    {                                                                          \
        if (!(expression))                                                     \
        {                                                                      \
            Test::fail(#expression, __FILE__, __LINE__);                       \
        }                                                                      \
    } while (false)

#define EXPECT_FALSE(expression) EXPECT_TRUE(!(expression))

#define EXPECT_EQ(actual, expected)                                            \
    Test::expectEqual(                                                         \
        actual,                                                                \
        expected,                                                              \
        #actual,                                                               \
        #expected,                                                             \
        __FILE__,                                                              \
        __LINE__)

#define EXPECT_NEAR(actual, expected, tolerance)                               \
    Test::expectNear(                                                          \
        actual,                                                                \
        expected,                                                              \
        tolerance,                                                             \
        #actual,                                                               \
        #expected,                                                             \
        __FILE__,                                                              \
        __LINE__)
