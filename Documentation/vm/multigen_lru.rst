=====================
Multigenerational LRU
=====================

Quick Start
===========
Build Options
-------------
:Required: Set ``CONFIG_LRU_GEN=y``.

:Optional: Change ``CONFIG_NR_LRU_GENS`` to a number ``X`` to support
 a maximum of ``X`` generations.

:Optional: Set ``CONFIG_LRU_GEN_ENABLED=y`` to turn the feature on by
 default.

Runtime Options
---------------
:Required: Write ``1`` to ``/sys/kernel/mm/lru_gen/enable`` if the
 feature was not turned on by default.

:Optional: Change ``/sys/kernel/mm/lru_gen/spread`` to a number ``N``
 to spread pages out across ``N+1`` generations. ``N`` must be less
 than ``X``. Larger values make the background aging more aggressive.

:Optional: Read ``/sys/kernel/debug/lru_gen`` to verify the feature.
 This file has the following output:

::

  memcg  memcg_id  memcg_path
    node  node_id
      min_gen  birth_time  anon_size  file_size
      ...
      max_gen  birth_time  anon_size  file_size

Given a memcg and a node, ``min_gen`` is the oldest generation
(number) and ``max_gen`` is the youngest. Birth time is in
milliseconds. Anon and file sizes are in pages.

Recipes
-------
:Android on ARMv8.1+: ``X=4``, ``N=0``

:Android on pre-ARMv8.1 CPUs: Not recommended due to the lack of
 ``ARM64_HW_AFDBM``

:Laptops running Chrome on x86_64: ``X=7``, ``N=2``

:Working set estimation: Write ``+ memcg_id node_id gen [swappiness]``
 to ``/sys/kernel/debug/lru_gen`` to account referenced pages to
 generation ``max_gen`` and create the next generation ``max_gen+1``.
 ``gen`` must be equal to ``max_gen`` in order to avoid races. A swap
 file and a non-zero swappiness value are required to scan anon pages.
 If swapping is not desired, set ``vm.swappiness`` to ``0`` and
 overwrite it with a non-zero ``swappiness``.

:Proactive reclaim: Write ``- memcg_id node_id gen [swappiness]
 [nr_to_reclaim]`` to ``/sys/kernel/debug/lru_gen`` to evict
 generations less than or equal to ``gen``. ``gen`` must be less than
 ``max_gen-1`` as ``max_gen`` and ``max_gen-1`` are active generations
 and therefore protected from the eviction. ``nr_to_reclaim`` can be
 used to limit the number of pages to be evicted. Multiple command
 lines are supported, so does concatenation with delimiters ``,`` and
 ``;``.

Workflow
========
Evictable pages are divided into multiple generations for each
``lruvec``. The youngest generation number is stored in ``max_seq``
for both anon and file types as they are aged on an equal footing. The
oldest generation numbers are stored in ``min_seq[2]`` separately for
anon and file types as clean file pages can be evicted regardless of
swap and write-back constraints. Generation numbers are truncated into
``ilog2(CONFIG_NR_LRU_GENS)+1`` bits in order to fit into
``page->flags``. The sliding window technique is used to prevent
truncated generation numbers from overlapping. Each truncated
generation number is an index to an array of per-type and per-zone
lists. Evictable pages are added to the per-zone lists indexed by
``max_seq`` or ``min_seq[2]`` (modulo ``CONFIG_NR_LRU_GENS``),
depending on whether they are being faulted in or read ahead. The
workflow comprises two conceptually independent functions: the aging
and the eviction.

Aging
-----
The aging produces young generations. Given an ``lruvec``, the aging
scans page tables for referenced pages of this ``lruvec``. Upon
finding one, the aging updates its generation number to ``max_seq``.
After each round of scan, the aging increments ``max_seq``. The aging
maintains either a system-wide ``mm_struct`` list or per-memcg
``mm_struct`` lists, and it only scans page tables of processes that
have been scheduled since the last scan. Since scans are differential
with respect to referenced pages, the cost is roughly proportional to
their number.

Eviction
--------
The eviction consumes old generations. Given an ``lruvec``, the
eviction scans the pages on the per-zone lists indexed by either of
``min_seq[2]``. It selects a type according to the values of
``min_seq[2]`` and swappiness. During a scan, the eviction either
sorts or isolates a page, depending on whether the aging has updated
its generation number. When it finds all the per-zone lists are empty,
the eviction increments ``min_seq[2]`` indexed by this selected type.
The eviction triggers the aging when both of ``min_seq[2]`` reaches
``max_seq-1``, assuming both anon and file types are reclaimable.

Rationale
=========
Characteristics of cloud workloads
----------------------------------
With cloud storage gone mainstream, the role of local storage has
diminished. For most of the systems running cloud workloads, anon
pages account for the majority of memory consumption and page cache
contains mostly executable pages. Notably, the portion of the unmapped
is negligible.

As a result, swapping is necessary to achieve substantial memory
overcommit. And the ``rmap`` is the hottest in the reclaim path
because its usage is proportional to the number of scanned pages,
which on average is many times the number of reclaimed pages.

With ``zram``, a typical ``kswapd`` profile on v5.11 looks like:

::

  31.03%  page_vma_mapped_walk
  25.59%  lzo1x_1_do_compress
   4.63%  do_raw_spin_lock
   3.89%  vma_interval_tree_iter_next
   3.33%  vma_interval_tree_subtree_search

And with real swap, it looks like:

::

  45.16%  page_vma_mapped_walk
   7.61%  do_raw_spin_lock
   5.69%  vma_interval_tree_iter_next
   4.91%  vma_interval_tree_subtree_search
   3.71%  page_referenced_one

Limitations of the Current Implementation
-----------------------------------------
Notion of the Active/Inactive
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
For servers equipped with hundreds of gigabytes of memory, the
granularity of the active/inactive is too coarse to be useful for job
scheduling. And false active/inactive rates are relatively high.

For phones and laptops, the eviction is biased toward file pages
because the selection has to resort to heuristics as direct
comparisons between anon and file types are infeasible.

For systems with multiple nodes and/or memcgs, it is impossible to
compare ``lruvec``\s based on the notion of the active/inactive.

Incremental Scans via the ``rmap``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Each incremental scan picks up at where the last scan left off and
stops after it has found a handful of unreferenced pages. For most of
the systems running cloud workloads, incremental scans lose the
advantage under sustained memory pressure due to high ratios of the
number of scanned pages to the number of reclaimed pages. On top of
that, the ``rmap`` has poor memory locality due to its complex data
structures. The combined effects typically result in a high amount of
CPU usage in the reclaim path.

Benefits of the Multigenerational LRU
-------------------------------------
Notion of Generation Numbers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The notion of generation numbers introduces a quantitative approach to
memory overcommit. A larger number of pages can be spread out across
configurable generations, and thus they have relatively low false
active/inactive rates. Each generation includes all pages that have
been referenced since the last generation.

Given an ``lruvec``, scans and the selections between anon and file
types are all based on generation numbers, which are simple and yet
effective. For different ``lruvec``\s, comparisons are still possible
based on birth times of generations.

Differential Scans via Page Tables
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Each differential scan discovers all pages that have been referenced
since the last scan. Specifically, it walks the ``mm_struct`` list
associated with an ``lruvec`` to scan page tables of processes that
have been scheduled since the last scan. The cost of each differential
scan is roughly proportional to the number of referenced pages it
discovers. Unless address spaces are extremely sparse, page tables
usually have better memory locality than the ``rmap``. The end result
is generally a significant reduction in CPU usage, for most of the
systems running cloud workloads.

To-do List
==========
KVM Optimization
----------------
Support shadow page table walk.

NUMA Optimization
-----------------
Add per-node RSS for ``should_skip_mm()``.

Refault Tracking Optimization
-----------------------------
Use generation numbers rather than LRU positions in
``workingset_eviction()`` and ``workingset_refault()``.
