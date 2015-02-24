// Copyright (c) 2014, Institute of Computer Engineering (ITI), Universität zu Lübeck
// Jan Frost, Jan Helge Klüssendorff
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or
//    other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may
//    be used to endorse or promote products derived from this software without
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_OPTIMIZER
#define G2O_OPTIMIZER

#include <graph_slam_tools/optimization/graph_optimizer.h>

#include <boost/bimap.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef boost::bimaps::bimap<std::string,g2o::HyperGraph::Edge*> g2o_edge_bimap;
typedef boost::bimaps::bimap<std::string,g2o::HyperGraph::Vertex*> g2o_vertex_bimap;

class G2oOptimizer : public GraphOptimizer
{
public:
    G2oOptimizer();
    ~G2oOptimizer();

protected:
    void addGraphImpl(SlamGraph &graph);
    void storeImpl(SlamGraph &graph);
    void optimizeImpl();

    void clear();
    void addVertex(const SlamNode &node);
    void addEdge(std::string id, SlamGraph &graph);
    void setFixedNodes();

    g2o::SparseOptimizer optimizer_;
    SlamLinearSolver *linear_solver_;
    SlamBlockSolver *block_solver_;
    g2o::OptimizationAlgorithm *solver_;

    g2o_vertex_bimap node_id_to_g2o_map_;
    g2o_edge_bimap edge_id_to_g2o_map_;

    int optimizer_count_;
};

#endif
