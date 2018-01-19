#include <glog/logging.h>
#include <v4r/recognition/ghv_opt.h>

namespace v4r {

// template<typename PointT>
// bool
// GHVmove_activate<PointT>::operator== (const mets::mana_move& m) const
//{
//    try
//    {
//        const GHVmove_activate& mm = dynamic_cast<const GHVmove_activate&> (m);
//        return mm.index_ == index_;
//    }
//    catch (std::bad_cast & bc)
//    {
//        std::cerr << "bad cast:" << bc.what() << "\n";
//        return false;
//    }
//}

// template<typename PointT>
// bool
// GHVmove_deactivate<PointT>::operator== (const mets::mana_move& m) const
//{
//    try
//    {
//        const GHVmove_deactivate& mm = dynamic_cast<const GHVmove_deactivate&> (m);
//        return mm.index_ == index_;
//    }
//    catch(std::bad_cast & bc)
//    {
//        std::cerr << "bad cast:" << bc.what() << "\n";
//        return false;
//    }
//}

template <typename PointT>
void GHVreplace_hyp_move<PointT>::apply(mets::feasible_solution& s) const {
  GHVSAModel<PointT>& model = dynamic_cast<GHVSAModel<PointT>&>(s);
  CHECK(model.solution_[active_id_] && !model.solution_[inactive_id_]);
  model.solution_.flip(active_id_);
  model.solution_.flip(inactive_id_);
  model.apply();
}

template <typename PointT>
mets::gol_type GHVreplace_hyp_move<PointT>::apply_and_evaluate(mets::feasible_solution& cs) {
  GHVSAModel<PointT>& model = dynamic_cast<GHVSAModel<PointT>&>(cs);
  CHECK(model.solution_[active_id_] && !model.solution_[inactive_id_]);
  model.solution_.flip(active_id_);
  model.solution_.flip(inactive_id_);
  return model.apply_and_evaluate();
}

template <typename PointT>
mets::gol_type GHVreplace_hyp_move<PointT>::evaluate(const mets::feasible_solution& cs) const {
  GHVSAModel<PointT> model;
  model.copy_from(cs);
  CHECK(model.solution_[active_id_] && !model.solution_[inactive_id_]);
  model.solution_.flip(active_id_);
  model.solution_.flip(inactive_id_);
  return model.evaluate();
}

///////////////////////////////////////////////////////////////
///////////// move manager ////////////////////////////////////
///////////////////////////////////////////////////////////////

template <typename PointT>
void GHVmove_manager<PointT>::refresh(mets::feasible_solution& s) {
  GHVSAModel<PointT>& model = dynamic_cast<GHVSAModel<PointT>&>(s);
  boost::dynamic_bitset<> crt_solution = model.opt_->getSolution();

  size_t inactive_hypotheses = crt_solution.size() - crt_solution.count();

  moves_m_.clear();

  if (use_replace_moves_)
    moves_m_.resize(inactive_hypotheses + crt_solution.size() * crt_solution.size());
  else
    moves_m_.resize(inactive_hypotheses);

  size_t kept = 0;
  for (size_t i = 0; i < crt_solution.size(); i++) {
    if (!crt_solution[i])
      moves_m_[kept++].reset(new GHVmove<PointT>(i));
    //        else
    //            moves_m_[kept++].reset( new GHVmove_deactivate<PointT> (i, crt_solution.size()) );
  }

  if (use_replace_moves_) {
    // based on s and the explained point intersection, create some replace_hyp_move
    // go through s and select active hypotheses and non-active hypotheses
    // check for each pair if the intersection is big enough
    // if positive, create a replace_hyp_move that will deactivate the act. hyp and activate the other one
    // MAYBE it would be interesting to allow this changes when the temperature is low or
    // there has been some iterations without an improvement
    for (size_t i = 0; i < crt_solution.size(); i++) {
      for (size_t j = 0; j < crt_solution.size(); j++)  // i active, j inactive
      {
        if (crt_solution[i] && !crt_solution[j] && intersection_cost_(i, j) > std::numeric_limits<float>::epsilon())
          moves_m_[kept++].reset(new GHVreplace_hyp_move<PointT>(i, j, crt_solution.size()));
      }
    }
  }

  moves_m_.resize(kept);
  //    std::random_shuffle (moves_m_.begin (), moves_m_.end ()); ///TODO: Is this relevant?
}

template class V4R_EXPORTS GHVmove_manager<pcl::PointXYZRGB>;
}
