# Multi-Agent Flocking with Cooperation-Competition Evolution

This repository contains the MATLAB source code for the simulation results presented in the manuscript **"Flocking Behavior for Multi-Agent Systems with Cooperation-Competition Evolution"**.

## Overview

The provided scripts simulate the collective behavior of multi-agent systems (MAS) where interaction weights evolve dynamically based on real-time state discrepancies. The model characterizes the transition between cohesive flocking and bipartite subgroup formation.

## Files Description

- `flocking_CC_leader.m`:
  - **Scenario**: Leader-follower flocking.
  - **Features**: Implements an extended system with $2n$ nodes to analyze the augmented digraph. It demonstrates how followers converge to the leader's state through dynamic weight transitions.
- `Flocking_CC_leaderless.m`:
  - **Scenario**: Leaderless flocking and bipartite formation.

  - **Features**: Simulates the system's evolution under different cooperation ranges ($\epsilon$). It illustrates the emergence of velocity consensus or bipartite synchronization.
