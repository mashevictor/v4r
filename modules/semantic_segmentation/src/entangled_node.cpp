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
 * @file   entangled_node.cpp
 * @author Daniel Wolf (wolf@acin.tuwien.ac.at)
 * @date   2017
 * @brief  .
 *
 */

#include <v4r/semantic_segmentation/entangled_node.h>

namespace v4r
{

    EntangledForestNode::EntangledForestNode() : mParent(nullptr), mOnFrontier(true), mIsSplitNode(false), mSplitFeature(nullptr), mDepth(-1),
                   mLeftChildIdx(-1), mRightChildIdx(-1)
    {

    }

// constructor for root node
    EntangledForestNode::EntangledForestNode(EntangledForestData *data) : mParent(nullptr), mOnFrontier(true), mIsSplitNode(false), mSplitFeature(nullptr), mDepth(0),
                             mLeftChildIdx(-1), mRightChildIdx(-1)
    {
        // set begin and end iterator
        data->GetBeginAndEndIterator(mTrainingDataIterators.first, mTrainingDataIterators.second);
        data->CalculateLabelDistributions(mTrainingDataIterators.first, mTrainingDataIterators.second,
                                          mAbsLabelDistribution, mLabelDistribution);
    }

// constructor for child node
    EntangledForestNode::EntangledForestNode(EntangledForestData *data, ClusterIdxItr dataStart, ClusterIdxItr dataEnd, EntangledForestNode *parent) : mParent(parent),
                                                                                           mOnFrontier(true),
                                                                                           mIsSplitNode(false),
                                                                                           mSplitFeature(nullptr),
                                                                                           mDepth(parent->GetDepth() +
                                                                                                 1), mLeftChildIdx(-1),
                                                                                           mRightChildIdx(-1)
    {
        mTrainingDataIterators.first = dataStart;
        mTrainingDataIterators.second = dataEnd;

        data->CalculateLabelDistributions(mTrainingDataIterators.first, mTrainingDataIterators.second,
                                          mAbsLabelDistribution, mLabelDistribution);
    }

    void EntangledForestNode::SetRandomGenerator(std::mt19937 *randomGenerator)
    {
        if (mIsSplitNode)
        {
            mSplitFeature->SetRandomGenerator(randomGenerator);
        }
    }

    int EntangledForestNode::evaluate(EntangledForestData *data, int imageIdx, int clusterIdx)
    {
        return mSplitFeature->evaluateInference(data, imageIdx, clusterIdx) ? mRightChildIdx : mLeftChildIdx;
    }

    void EntangledForestNode::SetAsLeafNode()
    {
        mIsSplitNode = false;
        mOnFrontier = false;
    }

    void EntangledForestNode::Clone(EntangledForestNode *n, EntangledForestTree *newTree)
    {
        n->mParent = mParent;
        n->mDepth = mDepth;
        n->mLeftChildIdx = mLeftChildIdx;
        n->mRightChildIdx = mRightChildIdx;
        n->mLabelDistribution = mLabelDistribution;
        n->mAbsLabelDistribution = mAbsLabelDistribution;
        n->mIsSplitNode = mIsSplitNode;
        n->mOnFrontier = mOnFrontier;

        if (mIsSplitNode)
            n->mSplitFeature = mSplitFeature->Clone(newTree);
    }

    void EntangledForestNode::Split(EntangledForestData *data, EntangledForestSplitFeature *f, EntangledForestNode **leftChild, EntangledForestNode **rightChild)
    {
        mOnFrontier = false;
        mIsSplitNode = true; // node changes to split node
        mSplitFeature = f->Clone();
        ClusterIdxItr divider = SplitOnFeature(f, data, mTrainingDataIterators.first, mTrainingDataIterators.second);

        (*leftChild) = new EntangledForestNode(data, mTrainingDataIterators.first, divider, this); // this->depth);
        (*rightChild) = new EntangledForestNode(data, divider, mTrainingDataIterators.second, this);
    }

    ClusterIdxItr EntangledForestNode::SplitOnFeature(EntangledForestSplitFeature *f, EntangledForestData *data, ClusterIdxItr start, ClusterIdxItr end)
    {
        auto i = start;
        auto j = end - 1;
        ClusterIdx swapidx;

        while (i != j)
        {
            if (f->evaluateInference(data, (*i)[0], (*i)[1]))
            {
                swapidx = *j;
                *j = *i;
                *i = swapidx;
                j--;
            }
            else
            {
                i++;
            }
        }

        // return iterator to first element of "right" group to mark the split element
        return f->evaluateInference(data, (*i)[0], (*i)[1]) ? i : i + 1;
    }

    bool EntangledForestNode::IsDescendantOf(EntangledForestNode *n)
    {
        EntangledForestNode *p = this;

        if (p == n)
        {
            return false;
        }

        int startDepth = mDepth;
        int targetDepth = n->GetDepth();

        for (int i = startDepth; i > targetDepth; --i)
        {
            p = p->GetParent();
        }

        return p == n;
    }

    double EntangledForestNode::GetWeightedNrOfPoints(std::vector<double> &classWeights)
    {
        double sum(0.0);

        for (unsigned int i = 0; i < mAbsLabelDistribution.size(); ++i)
        {
            sum += ((double) mAbsLabelDistribution[i]) / classWeights[i];
        }
        return sum;
    }

    void EntangledForestNode::ClearSplitNodeLabelDistribution()
    {
        if (mIsSplitNode)
        {
            mLabelDistribution.clear();
            mAbsLabelDistribution.clear();
        }
    }

    void EntangledForestNode::ResetLabelDistribution()
    {
        mLabelDistribution.assign(mLabelDistribution.size(), 0.0f);
        mAbsLabelDistribution.assign(mAbsLabelDistribution.size(), 0);
    }

    void EntangledForestNode::AddToAbsLabelDistribution(int labelIdx)
    {
        mAbsLabelDistribution[labelIdx]++;
    }

    void EntangledForestNode::UpdateLabelDistribution(std::vector<int> labels, std::map<int, unsigned int> &pointsPerLabel)
    {
        float sum = 0;

        for (unsigned int i = 0; i < labels.size(); ++i)
            mLabelDistribution[i] = ((float) mAbsLabelDistribution[i]) / ((float) pointsPerLabel[labels[i]]);

        for (unsigned int i = 0; i < mLabelDistribution.size(); ++i)
            sum += mLabelDistribution[i];

        for (unsigned int i = 0; i < mAbsLabelDistribution.size(); ++i)
            mLabelDistribution[i] /= sum;
    }


    EntangledForestNode *EntangledForestNode::GetParent()
    {
        return mParent;
    }

    EntangledForestSplitFeature *EntangledForestNode::GetSplitFeature()
    {
        return mSplitFeature;
    }

    void EntangledForestNode::SetParent(EntangledForestNode *par)
    {
        mParent = par;
    }

    bool EntangledForestNode::IsTopClass(int classIdx)
    {
        return std::distance(mLabelDistribution.begin(),
                             std::max_element(mLabelDistribution.begin(), mLabelDistribution.end())) == classIdx;
    }

    bool EntangledForestNode::IsAmongTopN(unsigned int classIdx, unsigned int N)
    {
        unsigned int larger = 0;
        double compareWith = mLabelDistribution[classIdx];

        for (unsigned int i = 0; i < mLabelDistribution.size(); ++i)
        {
            if (i != classIdx && mLabelDistribution[i] > compareWith)
            {
                if (++larger >= N)
                {
                    return false;
                }
            }
        }

        return true;
    }


    void EntangledForestNode::ApplyClusterLabelDistribution(std::vector<double> &labeldist)
    {
        std::copy(mLabelDistribution.begin(), mLabelDistribution.end(), labeldist.begin());
    }

    void EntangledForestNode::UpdateLabelDistribution(std::vector<double> &labeldist)
    {
        std::copy(labeldist.begin(), labeldist.end(), mLabelDistribution.begin());
    }

    EntangledForestNode::~EntangledForestNode()
    {
        delete mSplitFeature;
    }

}
