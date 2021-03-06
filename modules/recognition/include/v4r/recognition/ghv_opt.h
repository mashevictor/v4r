/****************************************************************************
**
** Copyright (C) 2017 TU Wien, ACIN, Vision 4 Robotics (V4R) group
** Contact: v4r.acin.tuwien.ac.at
**
** This file is part of V4R
**
** V4R is distributed under dual licenses - GPLv3 or closed source.
**
** GNU General Public License Usage
** V4R is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published
** by the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** V4R is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** Please review the following information to ensure the GNU General Public
** License requirements will be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
**
** Commercial License Usage
** If GPL is not suitable for your project, you must purchase a commercial
** license to use V4R. Licensees holding valid commercial V4R licenses may
** use this file in accordance with the commercial license agreement
** provided with the Software or, alternatively, in accordance with the
** terms contained in a written agreement between you and TU Wien, ACIN, V4R.
** For licensing terms and conditions please contact office<at>acin.tuwien.ac.at.
**
**
** The copyright holder additionally grants the author(s) of the file the right
** to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of their contributions without any restrictions.
**
****************************************************************************/

/**
 * @file ghv_opt.h
 * @author Aitor Aldoma (aldoma@acin.tuwien.ac.at), Thomas Faeulhammer (faeulhammer@acin.tuwien.ac.at)
 * @date 2013
 * @brief
 *
 */

#pragma once

#include <v4r/core/macros.h>
#include <v4r/recognition/hypotheses_verification.h>
#include <boost/dynamic_bitset.hpp>
#include <boost/function.hpp>
#include <boost/random.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <iostream>
#include <metslib/mets.hh>

namespace v4r {
template <typename PointT>
class HypothesisVerification;

template <typename PointT>
class V4R_EXPORTS GHVSAModel : public mets::evaluable_solution {
  typedef HypothesisVerification<PointT> SAOptimizerT;

 public:
  typedef boost::shared_ptr<GHVSAModel<PointT>> Ptr;
  typedef boost::shared_ptr<GHVSAModel<PointT> const> ConstPtr;

  SAOptimizerT* opt_;
  mets::gol_type cost_;
  boost::dynamic_bitset<> solution_;

  // Evaluates the current solution
  mets::gol_type cost_function() const {
    return cost_;
  }

  void copy_from(const mets::copyable& o) {
    const GHVSAModel& s = dynamic_cast<const GHVSAModel&>(o);
    opt_ = s.opt_;
    cost_ = s.cost_;
    solution_ = s.solution_;
  }

  void copy_from(const mets::feasible_solution& o) {
    const GHVSAModel& s = dynamic_cast<const GHVSAModel&>(o);
    opt_ = s.opt_;
    cost_ = s.cost_;
    solution_ = s.solution_;
  }

  mets::gol_type what_if(int /*index*/, bool /*val*/) const {
    return static_cast<mets::gol_type>(0);
  }

  mets::gol_type apply_and_evaluate() {
    cost_ = evaluate();  // this will update the state of the solution
    apply();
    return cost_;
  }

  mets::gol_type evaluate() {
    return opt_->evaluateSolution(solution_);
  }

  void apply() {
    opt_->applySolution(solution_);
    cost_ = opt_->evaluateSolution(solution_);
  }

  //    void
  //    unapply (size_t index)
  //    {
  //        cost_ = opt_->evaluateSolution (index); //this will udpate the cost function in opt_
  //    }

  void setSolution(const boost::dynamic_bitset<>& sol) {
    solution_ = sol;
  }

  void setOptimizer(SAOptimizerT* opt) {
    opt_ = opt;
  }
};

/**
   * @brief Represents a generic move from which all move types should inherit
   */
class V4R_EXPORTS GHVgeneric_move : public mets::mana_move {
 public:
  virtual mets::gol_type evaluate(const mets::feasible_solution& cs) const = 0;
  virtual mets::gol_type apply_and_evaluate(mets::feasible_solution& cs) = 0;
  virtual void apply(mets::feasible_solution& s) const = 0;
  virtual void unapply(mets::feasible_solution& s) const = 0;
  virtual mets::clonable* clone() const = 0;
  virtual size_t hash() const = 0;
  virtual bool operator==(const mets::mana_move&) const = 0;
};

/**
   * @brief Represents a move that deactivates an active hypothesis replacing it by an inactive one
   * Such moves should be done when the temperature is low
   * It is based on the intersection of explained points between hypothesis
   */
template <typename PointT>
class V4R_EXPORTS GHVreplace_hyp_move : public GHVgeneric_move {
  size_t active_id_, inactive_id_;  // i_ is an active hypothesis, j_ is an inactive hypothesis
  size_t sol_size_;

 public:
  GHVreplace_hyp_move(size_t active_id, size_t inactive_id, size_t sol_size)
  : active_id_(active_id), inactive_id_(inactive_id), sol_size_(sol_size) {}

  mets::gol_type evaluate(const mets::feasible_solution& cs) const;

  mets::gol_type apply_and_evaluate(mets::feasible_solution& cs);

  void apply(mets::feasible_solution& s) const;

  void unapply(mets::feasible_solution& s) const {
    (void)s;
    // go back
    throw std::runtime_error("Unapply is not implemented right now!");
    //        GHVSAModel<PointT>& model = dynamic_cast<GHVSAModel<PointT>&> (s);
    //        model.unapply (inactive_id_, !model.solution_[inactive_id_]);
    //        model.unapply (active_id_, !model.solution_[active_id_]);
  }

  mets::clonable* clone() const {
    GHVreplace_hyp_move* m = new GHVreplace_hyp_move(active_id_, inactive_id_, sol_size_);
    return static_cast<mets::clonable*>(m);
  }

  size_t hash() const {
    return sol_size_ + sol_size_ * active_id_ + inactive_id_;
  }

  bool operator==(const mets::mana_move& m) const {
    const GHVreplace_hyp_move& mm = dynamic_cast<const GHVreplace_hyp_move&>(m);
    return (mm.active_id_ == active_id_) && (mm.inactive_id_ == inactive_id_);
  }
};

/**
   * @brief Represents a move, activate/deactivate an hypothesis
   */
template <typename PointT>
class GHVmove : public GHVgeneric_move {
  size_t index_;

 public:
  GHVmove(size_t i) : index_(i) {}

  size_t getIndex() {
    return index_;
  }

  mets::gol_type evaluate(const mets::feasible_solution& cs) const {
    // mets::copyable copyable = dynamic_cast<mets::copyable> (&cs);
    GHVSAModel<PointT> model;
    model.copy_from(cs);
    model.solution_.flip(index_);
    return model.evaluate();
  }

  mets::gol_type apply_and_evaluate(mets::feasible_solution& cs) {
    GHVSAModel<PointT>& model = dynamic_cast<GHVSAModel<PointT>&>(cs);
    model.solution_.flip(index_);
    return model.apply_and_evaluate();
  }

  void apply(mets::feasible_solution& s) const {
    GHVSAModel<PointT>& model = dynamic_cast<GHVSAModel<PointT>&>(s);
    model.solution_.flip(index_);
    model.apply();
  }

  void unapply(mets::feasible_solution& s) const {
    (void)s;
    throw std::runtime_error("Unapply is not implemented right now!");
    //    GHVSAModel<PointT>& model = dynamic_cast<GHVSAModel<PointT>&> (s);
    //    model.unapply (index_, !model.solution_[index_]);
  }

  mets::clonable* clone() const {
    GHVmove* m = new GHVmove(index_);
    return static_cast<mets::clonable*>(m);
  }

  size_t hash() const {
    return index_;
  }

  bool operator==(const mets::mana_move& m) const {
    std::cerr << "Going to cast move, should not happen" << std::endl;
    const GHVmove& mm = dynamic_cast<const GHVmove&>(m);
    return mm.index_ == index_;
  }
};

template <typename PointT>
class V4R_EXPORTS GHVmove_manager {
 private:
  bool use_replace_moves_;

 public:
  std::vector<boost::shared_ptr<GHVgeneric_move>> moves_m_;
  Eigen::MatrixXf intersection_cost_;
  typedef typename std::vector<boost::shared_ptr<GHVgeneric_move>>::iterator iterator;

  iterator begin() {
    return moves_m_.begin();
  }
  iterator end() {
    return moves_m_.end();
  }

  GHVmove_manager(bool rp_moves = true) : use_replace_moves_(rp_moves) {}

  void setIntersectionCost(const Eigen::MatrixXf& intersection_cost) {
    intersection_cost_ = intersection_cost;
  }

  void UseReplaceMoves(bool rp_moves = true) {
    use_replace_moves_ = rp_moves;
  }

  void refresh(mets::feasible_solution& s);
};

template <typename PointT>
class V4R_EXPORTS GHVCostFunctionLogger : public mets::solution_recorder {
  std::vector<double> costs_;
  std::vector<double> costs_history_;
  size_t times_evaluated_;
  boost::function<void(const boost::dynamic_bitset<>&, double, size_t)> visualize_function_;

 public:
  GHVCostFunctionLogger();

  GHVCostFunctionLogger(mets::evaluable_solution& best)
  : mets::solution_recorder(), times_evaluated_(0), best_ever_m(best) {
    costs_.resize(1);
    costs_[0] = 0.f;
  }

  void setVisualizeFunction(boost::function<void(const boost::dynamic_bitset<>&, double, size_t)>& f) {
    visualize_function_ = f;
  }

  void writeToLog(std::ofstream& of) {
    of << times_evaluated_ << "\t\t" << costs_.size() << "\t\t" << costs_[costs_.size() - 1] << std::endl;
  }

  void writeEachCostToLog(std::ofstream& of) {
    for (size_t i = 0; i < costs_history_.size(); i++)
      of << costs_history_[i] << "\t";

    of << std::endl;
  }

  void addCost(float c) {
    costs_.push_back(c);
  }

  void addCostEachTimeEvaluated(float c) {
    costs_history_.push_back(c);
  }

  void increaseEvaluated() {
    times_evaluated_++;
  }

  size_t getTimesEvaluated() {
    return times_evaluated_;
  }

  size_t getAcceptedMovesSize() {
    return costs_.size();
  }

  bool accept(const mets::feasible_solution& sol) {
    const mets::evaluable_solution& s = dynamic_cast<const mets::evaluable_solution&>(sol);

    if (s.cost_function() < best_ever_m.cost_function())  // move accepted
    {
      best_ever_m.copy_from(s);
      const GHVSAModel<PointT>& ss = static_cast<const GHVSAModel<PointT>&>(sol);
      costs_.push_back(ss.cost_);

      if (visualize_function_)
        visualize_function_(ss.opt_->getSolution(), ss.cost_, times_evaluated_);

      return true;
    }
    return false;
  }

  /// @brief Returns the best solution found since the beginning.
  const mets::evaluable_solution& best_seen() const {
    return best_ever_m;
  }

  mets::gol_type best_cost() const {
    return best_ever_m.cost_function();
  }

  void setBestSolution(const boost::dynamic_bitset<>& sol) {
    GHVSAModel<PointT>& ss = static_cast<GHVSAModel<PointT>&>(best_ever_m);
    ss.solution_ = sol;
  }

 protected:
  mets::evaluable_solution& best_ever_m;  ///< Records the best solution
};
}
