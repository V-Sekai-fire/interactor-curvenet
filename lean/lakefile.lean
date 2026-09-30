import Lake
open Lake DSL

package Cassie where

-- Every dependency is pinned in a V-Sekai-fire repo or fork.
require LeanSlang from git
  "https://github.com/V-Sekai-fire/contract-lean-slang.git" @ "60532aef8ed70cc669ecab481182d0636c9e1ac3"

-- The curvenet stage's kernels (Cut 4): CASSIE's four editing-pipeline
-- kernels from entities-godot, see Cassie/CITATION.cff.
lean_lib Cassie

lean_exe emit_cassie where
  root := `EmitCassie
