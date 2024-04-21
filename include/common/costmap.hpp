#ifndef COSTMAP_TYPE_H
#define COSTMAP_TYPE_H

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#define COSTMAP_ALWAYS_INLINE 0


#include <ros/ros.h>
#include <limits>
#include "rosparams.h"
#include "opencv2/core/matx.hpp"
#include <QtGlobal>
#include <type_traits>
#include <math.h>
#include <complex>
#include "coord.hpp"
#include "describe/describe.hpp"

#define DEFAULT_COSTMAP_RESOLUTION 0.02
#define DEFAULT_COSTMAP_WIDTH 102
#define DEFAULT_COSTMAP_HEIGHT 152
#if !COSTMAP_ALWAYS_INLINE
#define COSTMAP_INLINE inline
#else
#define COSTMAP_INLINE Q_ALWAYS_INLINE
#endif


struct DPoint {
    int dx;
    int dy;
    float dist;
};

struct CostmapMetadata 
{
    CostmapMetadata():
      width(DEFAULT_COSTMAP_WIDTH), height(DEFAULT_COSTMAP_HEIGHT),
      elems(DEFAULT_COSTMAP_HEIGHT * DEFAULT_COSTMAP_WIDTH), resolution(DEFAULT_COSTMAP_RESOLUTION)
    {}
    CostmapMetadata(int width, int height, float resolution = DEFAULT_COSTMAP_RESOLUTION) :
    width(width), height(height), elems(height * width), resolution(resolution)
    {}
    CostmapMetadata(const CostmapMetadata &src) :
    width(src.width), height(src.height),elems(src.elems),resolution(src.resolution)
    {}
    CostmapMetadata& operator=(const CostmapMetadata& other) = default;
    int width;
    int height;
    int elems;
    float resolution;
};

template <typename T, typename U>
inline std::vector<T> copyVectorAs(const std::vector<U> &src) {
    return std::vector<T>(src.begin(), src.end());
}

struct InflateSettings {
    enum Algorithm {
        Linear = 0
    };
    float robot_safe_radius = 0.20;
};
DESCRIBE(InflateSettings, &_::robot_safe_radius)

template<typename T>
class MinMax {
    public:
    using type = typename std::conditional<std::is_floating_point<T>::value, signed long long, T>::type; 
};

template<class T, typename MinMax<T>::type maxVal = 100>
class Costmap
{
    static_assert(std::is_arithmetic<T>::value, "Costmap accepts only arithmetic types!");
public:
    Costmap() : 
        m_meta(), 
        m_cost_serialized(m_meta.elems, 0) 
    {
        updateSettings({});
    };
    Costmap(Costmap &&src) :
        m_meta(src.m_meta), 
        m_cost_serialized(std::move(src.m_cost_serialized))
    {
        updateSettings({});
    };
    Costmap(const Costmap &src) :
        m_meta(src.m_meta), 
        m_cost_serialized(src.m_cost_serialized)
    {
        updateSettings({});
    };
    Costmap(const std::vector<T> &src, const CostmapMetadata &meta) :
        m_meta(meta),
        m_cost_serialized(src)
    {
        updateSettings({});
    };
    Costmap(int width, int height, float resolution = DEFAULT_COSTMAP_RESOLUTION) :
        m_meta(width, height, resolution),
        m_cost_serialized(m_meta.elems, 0)
    {
        updateSettings({});
    };
    Costmap(const std::vector<T> &src, int width, int height, float resolution = DEFAULT_COSTMAP_RESOLUTION) :
        m_meta(width, height, resolution),
        m_cost_serialized(src)
    {
        updateSettings({});
    };
    Costmap(int width, int height) : 
        m_meta(width, height), 
        m_cost_serialized(m_meta.elems, 0)
    {
        updateSettings({});
    };
    Costmap(const CostmapMetadata &meta) : 
        m_meta(meta), 
        m_cost_serialized(m_meta.elems, 0)
    {
        updateSettings({});
    };

    template <typename U>
    Costmap& fillFromVector(U&& src) noexcept {
        m_cost_serialized = copyVectorAs<T>(src);
        for (auto &item : m_cost_serialized) {
            item = item > maxVal ? maxVal : item;
            item = item < 0 ? 0 : item;
        }
        return *this;
    }
    Costmap& operator=(const Costmap& other) = default;
    Costmap& operator=(Costmap&& other) = default;
    template <typename U, U rMaxVal = std::numeric_limits<U>::max()>
    Costmap& copyFrom(const Costmap<U, rMaxVal>& rhv) {
        m_cost_serialized = copyVectorAs<T>(rhv.data());
        m_meta = rhv.meta();
        m_dPoints = rhv.m_dPoints;
        m_inflSettings = rhv.m_inflSettings;
        m_inflationRadius = rhv.m_inflationRadius;
        return *this;
    }

    inline Costmap& copyFrom(Costmap&& rhv) {
        m_cost_serialized = std::move(rhv.data());
        m_meta = rhv.meta();
        m_dPoints = rhv.m_dPoints;
        m_inflSettings = rhv.m_inflSettings;
        m_inflationRadius = rhv.m_inflationRadius;
        return *this;
    }
    inline std::vector<T>& data() {return m_cost_serialized;}
    inline const std::vector<T>& data() const {return m_cost_serialized;}
    COSTMAP_INLINE void updateSettings(const InflateSettings &settings) {
        m_inflSettings = settings;
        m_inflationRadius = radiusFromMeters(m_inflSettings.robot_safe_radius);
        m_dPoints = createDPoints(m_inflSettings.robot_safe_radius);
    }
    COSTMAP_INLINE Costmap inflated() const noexcept {
        auto copy = *this;
        inflateInto(copy);
        return copy;
    }
    COSTMAP_INLINE void inflateInto(Costmap &target) const noexcept {
        if (target.size() != size()) {
                target.copyFrom(*this);
            }
            int x, y;
            auto csize = size();
            for (size_t i = 0; i < csize; ++i) {
                flatCoordTo2dUnsafe(i, x, y);
                inflateCoord(target, x, y, m_dPoints);
            }
        }
    //! Main algorithm
    COSTMAP_INLINE void inflateCoord(Costmap& result, int x, int y, const std::vector<DPoint> &dpoints) const {
        if (!atUnsafe(x, y)) return;
        if (isSurrounded(x, y)) return;
        for (const auto &dpoint : dpoints) {
            inflateCoordImpl(result, x + dpoint.dx, y + dpoint.dy, dpoint);
        }
    }
    COSTMAP_INLINE void inflateCoordImpl(Costmap& result, int x, int y, const DPoint& dpoint) const {
        auto coord = Coord{x, y};
        if (!coordIsValid(coord) || dpoint.dist > m_inflationRadius) return;
        auto was = result.at(coord);
        auto wanted = (m_inflationRadius - dpoint.dist) / m_inflationRadius * maxVal;
        if (wanted > was) {
            result.set(coord, wanted);
        }
    }

    COSTMAP_INLINE int radiusFromMeters(float meters) const noexcept {
        return meters / m_meta.resolution;
    }

    COSTMAP_INLINE void metersPosToCells(const float x, const float y, int &outx, int &outy) const noexcept {
        outx = x / m_meta.resolution;
        outy = y / m_meta.resolution;
    }

    COSTMAP_INLINE Coord metersPosToCells(const float x, const float y) const  {
        return {static_cast<int>(x / m_meta.resolution), static_cast<int>(y / m_meta.resolution)};
    }

    COSTMAP_INLINE Coord_<float> metersPosFromCells(const int x, const int y) const  {
        return {x * m_meta.resolution, y * m_meta.resolution};
    }

    COSTMAP_INLINE Coord_<float> metersPosFromCells(const Coord &cells) const  {
        return {cells.getX() * m_meta.resolution, cells.getY() * m_meta.resolution};
    }
    COSTMAP_INLINE void flatCoordTo2dUnsafe(const size_t coord, int &outx, int &outy) const  {
        outx = coord % m_meta.width;
        outy = coord / m_meta.width;
    }

    COSTMAP_INLINE Coord flatCoordTo2d(const size_t coord, bool *ok = nullptr) const  {
        if (coord < 0 || size() < coord) {
            if (ok) *ok = false;
            return {};
        }
        if (ok) *ok = true;
        return {coord % m_meta.width, coord / m_meta.width};
    }

    COSTMAP_INLINE size_t size() const noexcept {return m_cost_serialized.size();}
    inline bool isEmpty() const noexcept {return m_cost_serialized.empty();}
    COSTMAP_INLINE void resetValues() {
        std::fill(m_cost_serialized.begin(), m_cost_serialized.end(), 0);
    }
    COSTMAP_INLINE T operator()(int x, int y){return m_cost_serialized.at(m_meta.width * y + x);}
    COSTMAP_INLINE const T at(const Coord& coord) const {return m_cost_serialized.at(m_meta.width * coord.getY() + coord.getX());}
    COSTMAP_INLINE const T at(int x, int y) const {return m_cost_serialized.at(m_meta.width * y + x);}
    COSTMAP_INLINE const T atUnsafe(int x, int y) const noexcept {return m_cost_serialized[m_meta.width * y + x];}
    COSTMAP_INLINE const T operator[](const Coord& coord) const noexcept {return m_cost_serialized[m_meta.width * coord.getY() + coord.getX()];}
    COSTMAP_INLINE void set(const Coord& coord, T value) {
        m_cost_serialized.at(m_meta.width * coord.getY() + coord.getX()) = value;
    }
    COSTMAP_INLINE void set(int x, int y, T value) {
        m_cost_serialized.at(m_meta.width * y + x) = value;
    }
    COSTMAP_INLINE void setUnsafe(int x, int y, T value) noexcept {
        m_cost_serialized[m_meta.width * y + x] = value;
    }
    COSTMAP_INLINE bool coordIsValid(const Coord& coord) const noexcept {return coordIsValid(coord.getX(), coord.getY());}
    inline bool coordIsValid(int x, int y) const noexcept {return x < m_meta.width && x >= 0 &&  y < m_meta.height && y >= 0;}
    inline bool operator==(const Costmap &src) const noexcept {return m_cost_serialized == src.m_cost_serialized;}
    inline const CostmapMetadata &meta() const noexcept {return m_meta;}
    void setMeta(const CostmapMetadata& src) {m_meta = src;}
    // Interaction between two costmaps
    COSTMAP_INLINE Costmap operator+(const Costmap &src) const noexcept
    {
        Costmap result(*this);
        std::transform(result.data().begin(), result.data().end(), src.data().begin(), result.data().begin(), addNonRounding);
        return result;
    }
    COSTMAP_INLINE Costmap operator+(Costmap &&src) const noexcept
    {
        Costmap result(std::move(src));
        std::transform(result.data().begin(), result.data().end(), data().begin(), result.data().begin(), addNonRounding);
        return result;
    }
    COSTMAP_INLINE Costmap &operator+=(const Costmap &src) noexcept
    {
        std::transform(data().begin(), data().end(), src.data().begin(), data().begin(), addNonRounding);
        return *this;
    }
    COSTMAP_INLINE Costmap &addCosts(const Costmap &src) noexcept
    {
        return *this += src;
    }
    COSTMAP_INLINE Costmap &setAll(const Coord& coord, const std::vector<DPoint> &dpoints, T value) noexcept
    {
        for (int i = 0; i < dpoints.size(); ++i) {
            auto wanted = coord + Coord{dpoints[i].dx, dpoints[i].dy};
            if (coordIsValid(wanted)) {
                setUnsafe(wanted[0], wanted[1], value);
            }
        }
        return *this;
    }
    COSTMAP_INLINE void applyObject(const Coord& coord, float sizeMeters) noexcept {
        auto radius = radiusFromMeters(sizeMeters);
        for (int dx = 0; dx <= radius; ++dx) {
            for (int dy = 0; dy <= radius; ++dy) {
                if (sqrtf32(dx*dx + dy*dy) <= radius) {
                    if (coordIsValid(coord.getX() + dx, coord.getY() + dy)) {
                        set({coord.getX() + dx, coord.getY() + dy}, 100);
                    }
                    if (coordIsValid(coord.getX() - dx, coord.getY() + dy)) {
                        set({coord.getX() - dx, coord.getY() + dy}, 100);
                    }
                    if (coordIsValid(coord.getX() + dx, coord.getY() - dy)) {
                        set({coord.getX() + dx, coord.getY() - dy}, 100);
                    }
                    if (coordIsValid(coord.getX() - dx, coord.getY() - dy)) {
                        set({coord.getX() - dx, coord.getY() - dy}, 100);

                    }
                }
            }
        }
    }
    COSTMAP_INLINE std::vector<DPoint> createDPoints(float radiusMeters) {
        std::vector<DPoint> result;
        auto radius = radiusFromMeters(radiusMeters);
        result.reserve(radiusMeters * radiusMeters);
        for (int dx = 0; dx <= radius; ++dx) {
            for (int dy = 0; dy <= radius; ++dy) {
                auto dpoint = DPoint{dx, dy, sqrtf32(dx*dx + dy*dy)};
                if (dpoint.dist <= radius) {
                    result.push_back(dpoint);
                    result.push_back(DPoint{-dx, dy, dpoint.dist});
                    result.push_back(DPoint{dx, -dy, dpoint.dist});
                    result.push_back(DPoint{-dx, -dy, dpoint.dist});
                }
            }
        }
        return result;
    }
private:
    COSTMAP_INLINE bool isFilled(int x, int y) const noexcept {
        return coordIsValid(x, y) ? atUnsafe(x, y) : true;
    }
    COSTMAP_INLINE bool isSurrounded(int x, int y) const noexcept {
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (!isFilled(x + dx, y + dy)) {
                    return false;
                }
            }
        }
        return true;
    }
    COSTMAP_INLINE constexpr static T addNonRounding(T lhv, T rhv) noexcept {
        return (maxVal - lhv) > rhv ? (lhv + rhv < 0 ? 0 : (lhv + rhv)) : maxVal; 
    }
    CostmapMetadata m_meta;
    std::vector<T> m_cost_serialized;
    std::vector<DPoint> m_dPoints{};
    InflateSettings m_inflSettings{};
    float m_inflationRadius{1};
};

#endif // COSTMAP_TYPE_H
