//
// Created by uto_zzq on 23-8-15.
//

#ifndef UTO_PER_FS_SRC_PERIPHERAL_EXPAND_H_
#define UTO_PER_FS_SRC_PERIPHERAL_EXPAND_H_
#include "peripheral/defs.h"
#include "peripheral/utils.h"
#include "quads_intersect.h"
namespace fs
{
#if FS_CHECK(CFG_EXPAND_TABLE)
  class Expand
  {
  public:
    Expand(const Expand&)         = delete;
    void operator=(const Expand&) = delete;

    static Expand& instance();

    std::vector<cv::Point2f>                     getPolygonByOval(const ObjectState& ob, float dt, std::string& expand) const;
    std::vector<FSVec2f>                         assembleEgoBox(const ObjectState& ob) const;
    const std::vector<std::vector<fs::FSVec3f>>& getZoneBox() const { return zone_boxes_; };
    const std::vector<fs::FSVec2f>&              getSafeBox() const { return safe_boxes_; };
    const std::vector<fs::FSVec2f>&              getCautionBox() const { return caution_boxes_; };
    bool                                         isBoxIntersect(const std::vector<fs::FSVec2f>& box1, const std::vector<fs::FSVec2f>& box2) const
    {
      return quads_intersect_.quadsIntersect(box1, box2);
    }

  private:
    Expand();
    std::vector<cv::Point2f>              getPolygon(const std::vector<FSVec2f>& box_polygon_ego, float x, float y) const;
    std::pair<float, float>               getOvalXY(Zone zone_x, Zone zone_y, const ObjectState& ob) const;
    [[nodiscard]] std::pair<float, float> at(int zone_x, int zone_y, int label) const;

    static constexpr int X_ = static_cast<size_t>(SubLabel::EQUIPMENT) + 1;
    static constexpr int Y_ = static_cast<size_t>(Zone::IGNORANCE) + 1;

    float y_expand_map_[X_][Y_];
    float x_expand_map_[X_][Y_];

    // -- super parameters
    const float half_ego_length_ = 15.0f * 0.5f;
    const float half_ego_width_  = 2.8f * 0.5f;
    const float safe_x_          = half_ego_length_ + 15.f;
    const float safe_y_          = half_ego_width_ + 2.f;
    const float caution_x_       = half_ego_length_ + 25.f;
    const float caution_y_       = half_ego_width_ + 4.f;

    std::vector<std::vector<fs::FSVec3f>> zone_boxes_{};
    std::vector<fs::FSVec2f>              safe_boxes_{};
    std::vector<fs::FSVec2f>              caution_boxes_{};
    QuadsIntersect                        quads_intersect_;
  };
#else
  class Expand
  {
  public:
    Expand(const Expand&)         = delete;
    void operator=(const Expand&) = delete;

    static Expand& instance();

    std::vector<cv::Point2f>                     getPolygonByOval(const ObjectState& ob, float dt, std::string& expand) const;
    std::vector<FSVec2f>                         assembleEgoBox(const ObjectState& ob) const;
    const std::vector<std::vector<fs::FSVec3f>>& getZoneBox() const { return zone_boxes_; };

  private:
    Expand(){};
    std::vector<cv::Point2f> getPolygon(const std::vector<FSVec2f>& box_polygon_ego, float x, float y) const;

    std::vector<std::vector<fs::FSVec3f>>    zone_boxes_{};
    const std::map<const float, const float> EXPAND_SMALL_MOVING = {{0.F, 0.3F}, // -- distance v_error
                                                                    {5.F, 0.3F},
                                                                    {10.F, 0.4F},
                                                                    {15.F, 0.5F}};

    const std::map<const float, const float> EXPAND_SMALL_STOPPED = {{0.F, 0.4F}, // -- m
                                                                     {5.F, 0.8F},
                                                                     {20.F, 1.2F},
                                                                     {50.F, 1.5F}};

    const std::map<const float, const float> EXPAND_BIG_MOVING = {{0.F, 0.3F}, // -- distance v_error
                                                                  {5.F, 0.3F},
                                                                  {10.F, 0.4F},
                                                                  {15.F, 0.5F}};

    const std::map<const float, const float> EXPAND_BIG_STOPPED = {{0.F, 0.6F}, // -- m
                                                                   {5.F, 1.0F},
                                                                   {20.F, 1.2F},
                                                                   {50.F, 1.5F}};
  };
#endif
} // namespace fs

#endif //UTO_PER_FS_SRC_PERIPHERAL_EXPAND_H_
